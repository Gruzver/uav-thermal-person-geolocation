#!/usr/bin/env python3

"""
georeferencer.py

Nodo ROS2 que convierte las detecciones de personas (coordenadas en pixeles)
a ubicaciones GPS absolutas, sincronizando las detecciones con el estado del
dron mas cercano en el tiempo.

Suscripciones:
    /detection/persons      (drone_tracker_msgs/PersonDetectionArray)
    /telemetry/drone/state  (drone_tracker_msgs/DroneState)

Publicaciones:
    /gps/persons_location   (drone_tracker_msgs/PersonLocationArray)

El nodo mantiene un buffer circular de estados del dron para poder asociar
cada deteccion con la posicion y orientacion del dron en el momento exacto
en que se capturo el frame.
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from drone_tracker_msgs.msg import (
    DroneState,
    PersonDetectionArray,
    PersonLocation,
    PersonLocationArray,
)
from drone_tracker_utils import GeoCalculator, KalmanTracker


class GeoreferencingNode(Node):
    """
    Georreferencia personas detectadas usando la telemetria sincronizada del dron.

    Parametros ROS2:
        fov_horizontal          (float) FOV horizontal de la camara en grados (default: 50.5).
        max_person_distance     (float) Distancia maxima aceptable en metros (default: 1000.0).
        min_person_distance     (float) Distancia minima aceptable en metros (default: 10.0).
        drone_state_buffer_size (int)   Tamano del buffer circular de estados del dron (default: 20).
        max_sync_delta_ms       (float) Desfase maximo aceptable entre deteccion y estado del
                                        dron en milisegundos. Si se supera, el frame se descarta
                                        (default: 500.0).
        use_kalman              (bool)  Activa el filtro de Kalman para suavizar GPS (default: true).
        kalman_process_noise    (float) Varianza del ruido de proceso del filtro (default: 1e-10).
        kalman_measurement_noise (float) Varianza del ruido de medicion del filtro (default: 1e-8).
    """

    def __init__(self):
        super().__init__('georeferencing_node')

        self.declare_parameter('fov_horizontal', 50.5)
        self.declare_parameter('max_person_distance', 1000.0)
        self.declare_parameter('min_person_distance', 10.0)
        self.declare_parameter('drone_state_buffer_size', 20)
        self.declare_parameter('max_sync_delta_ms', 500.0)
        self.declare_parameter('use_kalman', True)
        self.declare_parameter('kalman_process_noise', 1e-10)
        self.declare_parameter('kalman_measurement_noise', 1e-8)

        self.fov_h = self.get_parameter('fov_horizontal').value
        self.max_distance = self.get_parameter('max_person_distance').value
        self.min_distance = self.get_parameter('min_person_distance').value
        self.buffer_size = self.get_parameter('drone_state_buffer_size').value
        self.max_sync_delta_s = self.get_parameter('max_sync_delta_ms').value / 1000.0
        self.use_kalman = self.get_parameter('use_kalman').value
        self.kalman_pn = self.get_parameter('kalman_process_noise').value
        self.kalman_mn = self.get_parameter('kalman_measurement_noise').value

        self.get_logger().info('Georeferencing Node')
        self.get_logger().info(f'  FOV horizontal:   {self.fov_h:.1f} deg')
        self.get_logger().info(f'  Rango de distancia: {self.min_distance:.1f} m - {self.max_distance:.1f} m')
        self.get_logger().info(f'  Buffer estados dron: {self.buffer_size} frames')
        self.get_logger().info(f'  Max desfase sincronizacion: {self.max_sync_delta_s * 1000:.0f} ms')
        self.get_logger().info(f'  Filtro de Kalman: {"activado" if self.use_kalman else "desactivado"}')

        self.geo_calc = GeoCalculator(fov_horizontal=self.fov_h)

        # Filtros de Kalman por track_id; se crean al primer avistamiento
        # y se eliminan cuando el track deja de aparecer en un frame.
        self._kalman_trackers: dict = {}

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub_detections = self.create_subscription(
            PersonDetectionArray,
            '/detection/persons',
            self._detections_callback,
            qos_sensor,
        )
        self.sub_drone_state = self.create_subscription(
            DroneState,
            '/telemetry/drone/state',
            self._drone_state_callback,
            qos_sensor,
        )
        self.pub_persons_location = self.create_publisher(
            PersonLocationArray, '/gps/persons_location', qos_pub
        )

        # Buffer circular de estados del dron para sincronizacion temporal
        self.drone_state_buffer: deque = deque(maxlen=self.buffer_size)

        self.frame_count = 0
        self.processed_count = 0
        self.rejected_count = 0
        self._timing_warnings = 0

        self.get_logger().info('Georeferencing Node listo.')

    def _drone_state_callback(self, msg: DroneState) -> None:
        """
        Almacena el estado del dron en el buffer circular.

        El estado se indexa por su timestamp en segundos para la sincronizacion
        posterior con las detecciones.

        Args:
            msg: Mensaje DroneState con posicion, altitud y angulos del dron.
        """
        timestamp = msg.timestamp.sec + msg.timestamp.nanosec / 1e9

        self.drone_state_buffer.append({
            'timestamp':    timestamp,
            'latitude':     msg.latitude,
            'longitude':    msg.longitude,
            'altitude_rel': msg.altitude_rel,
            'altitude_abs': msg.altitude_abs,
            'yaw':          msg.yaw,
            'pitch':        msg.pitch,
            'roll':         msg.roll,
        })

    def _get_drone_state_at(self, timestamp: float) -> dict | None:
        """
        Devuelve el estado del dron cuyo timestamp sea mas cercano al indicado.

        Si el buffer esta vacio o el estado mas cercano supera el umbral de
        sincronizacion, devuelve None.

        Args:
            timestamp: Tiempo objetivo en segundos (epoch).

        Returns:
            Diccionario con el estado del dron o None si no es posible sincronizar.
        """
        if not self.drone_state_buffer:
            return None

        closest = min(
            self.drone_state_buffer,
            key=lambda s: abs(s['timestamp'] - timestamp),
        )
        delta = abs(closest['timestamp'] - timestamp)

        if delta > self.max_sync_delta_s:
            self.get_logger().warning(
                f'Desfase de sincronizacion {delta * 1000:.0f} ms supera el limite '
                f'({self.max_sync_delta_s * 1000:.0f} ms). Frame descartado.'
            )
            return None

        if delta > 0.1:
            self._timing_warnings += 1
            # Throttle: primer aviso y luego cada 50
            if self._timing_warnings == 1 or self._timing_warnings % 50 == 0:
                self.get_logger().warning(
                    f'Desfase de sincronizacion elevado: {delta * 1000:.1f} ms '
                    f'(aviso #{self._timing_warnings})'
                )

        return closest

    def _detections_callback(self, msg: PersonDetectionArray) -> None:
        """
        Procesa un array de detecciones, obtiene el estado del dron sincronizado
        y calcula la ubicacion GPS de cada persona detectada.

        Las detecciones fuera del rango de distancia configurado se descartan.
        El resultado se publica como PersonLocationArray.

        Args:
            msg: Array de detecciones con coordenadas en espacio 640x512.
        """
        self.frame_count += 1
        detection_time = msg.timestamp.sec + msg.timestamp.nanosec / 1e9

        drone_state = self._get_drone_state_at(detection_time)
        if drone_state is None:
            self.get_logger().warning(f'Frame {self.frame_count}: sin estado del dron disponible para sincronizar.')
            return

        location_array = PersonLocationArray()
        location_array.timestamp = msg.timestamp

        current_ids: set = set()

        for detection in msg.detections:
            gps_location = self.geo_calc.pixel_to_gps(
                bbox_center_x=detection.centroid_x,
                bbox_center_y=detection.centroid_y,
                latitude_drone=drone_state['latitude'],
                longitude_drone=drone_state['longitude'],
                altitude_drone=drone_state['altitude_rel'],
                yaw=drone_state['yaw'],
                pitch=drone_state['pitch'],
                roll=drone_state['roll'],
            )

            if gps_location is None:
                self.rejected_count += 1
                self.get_logger().debug(f'ID {detection.track_id}: calculo GPS fallido.')
                continue

            if not (self.min_distance <= gps_location.distance_m <= self.max_distance):
                self.rejected_count += 1
                self.get_logger().debug(
                    f'ID {detection.track_id}: distancia {gps_location.distance_m:.1f} m '
                    f'fuera de rango [{self.min_distance:.1f}, {self.max_distance:.1f}].'
                )
                continue

            lat = gps_location.latitude
            lon = gps_location.longitude

            if self.use_kalman:
                tid = detection.track_id
                if tid not in self._kalman_trackers:
                    self._kalman_trackers[tid] = KalmanTracker(
                        lat, lon, detection_time,
                        process_noise=self.kalman_pn,
                        measurement_noise=self.kalman_mn,
                    )
                else:
                    self._kalman_trackers[tid].predict(detection_time)
                lat, lon = self._kalman_trackers[tid].update(lat, lon, detection_time)
                current_ids.add(tid)

            person_loc = PersonLocation()
            person_loc.track_id = detection.track_id
            person_loc.latitude = lat
            person_loc.longitude = lon
            person_loc.distance_m = gps_location.distance_m
            person_loc.confidence = detection.confidence
            person_loc.status = 'active'
            person_loc.timestamp = msg.timestamp
            location_array.persons.append(person_loc)
            self.processed_count += 1

        # Eliminar filtros de IDs que desaparecieron en este frame
        if self.use_kalman:
            stale_ids = set(self._kalman_trackers.keys()) - current_ids
            for tid in stale_ids:
                del self._kalman_trackers[tid]

        self.pub_persons_location.publish(location_array)

        if self.frame_count % 50 == 0:
            total = self.processed_count + self.rejected_count
            rejection_rate = (self.rejected_count / max(1, total)) * 100
            self.get_logger().info(
                f'Frame {self.frame_count} | '
                f'Personas publicadas: {len(location_array.persons)} | '
                f'Tasa de rechazo: {rejection_rate:.1f}%'
            )


def main(args=None):
    rclpy.init(args=args)
    node = GeoreferencingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

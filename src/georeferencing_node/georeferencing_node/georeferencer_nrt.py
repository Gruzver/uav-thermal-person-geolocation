#!/usr/bin/env python3

"""
georeferencer_nrt.py

Variante NRT (no real-time) del georeferencing_node.

Solo georreferencia cuando el dron lleva al menos `stability_window_s` segundos
estable en posicion y angulos. Elimina la sincronizacion por timestamp: usa
siempre el estado de telemetria mas reciente.

Suscripciones:
    /detection/persons      (drone_tracker_msgs/PersonDetectionArray)
    /telemetry/drone/state  (drone_tracker_msgs/DroneState)

Publicaciones:
    /gps/persons_location   (drone_tracker_msgs/PersonLocationArray)
"""

import math
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

# Metros por grado de latitud, usado para expresar la dispersion de posicion
# del dron en metros al evaluar la ventana de estabilidad.
_LAT_M_PER_DEG = 111_320.0


class GeoreferencingNodeNrt(Node):
    """
    Georreferencia personas detectadas solo cuando el dron esta estable.

    Parametros ROS2:
        fov_horizontal          (float) FOV horizontal de la camara en grados (default: 50.5).
        max_person_distance     (float) Distancia maxima aceptable en metros (default: 1000.0).
        min_person_distance     (float) Distancia minima aceptable en metros (default: 10.0).
        use_kalman              (bool)  Activa el filtro de Kalman (default: true).
        kalman_process_noise    (float) Varianza del ruido de proceso (default: 1e-10).
        kalman_measurement_noise (float) Varianza del ruido de medicion (default: 1e-8).
        stability_window_s      (float) Segundos que el dron debe estar estable (default: 4.0).
        stability_pos_std_m     (float) Max desv. estandar de posicion en metros (default: 2.0).
        stability_angle_std_deg (float) Max desv. estandar de angulos en grados (default: 1.0).
    """

    def __init__(self):
        super().__init__('georeferencing_node_nrt')

        self.declare_parameter('fov_horizontal', 50.5)
        self.declare_parameter('max_person_distance', 1000.0)
        self.declare_parameter('min_person_distance', 10.0)
        self.declare_parameter('use_kalman', True)
        self.declare_parameter('kalman_process_noise', 1e-10)
        self.declare_parameter('kalman_measurement_noise', 1e-8)
        self.declare_parameter('stability_window_s', 4.0)
        self.declare_parameter('stability_pos_std_m', 2.0)
        self.declare_parameter('stability_angle_std_deg', 1.0)

        self.fov_h = self.get_parameter('fov_horizontal').value
        self.max_distance = self.get_parameter('max_person_distance').value
        self.min_distance = self.get_parameter('min_person_distance').value
        self.use_kalman = self.get_parameter('use_kalman').value
        self.kalman_pn = self.get_parameter('kalman_process_noise').value
        self.kalman_mn = self.get_parameter('kalman_measurement_noise').value
        self.stability_window_s = self.get_parameter('stability_window_s').value
        self.stability_pos_std_m = self.get_parameter('stability_pos_std_m').value
        self.stability_angle_std_deg = self.get_parameter('stability_angle_std_deg').value

        self.get_logger().info('Georeferencing Node NRT')
        self.get_logger().info(f'  FOV horizontal:     {self.fov_h:.1f} deg')
        self.get_logger().info(f'  Rango de distancia: {self.min_distance:.1f} - {self.max_distance:.1f} m')
        self.get_logger().info(f'  Ventana estabilidad: {self.stability_window_s:.1f} s')
        self.get_logger().info(f'  Umbral posicion:    {self.stability_pos_std_m:.1f} m std')
        self.get_logger().info(f'  Umbral angulos:     {self.stability_angle_std_deg:.1f} deg std')
        self.get_logger().info(f'  Filtro de Kalman:   {"activado" if self.use_kalman else "desactivado"}')

        self.geo_calc = GeoCalculator(fov_horizontal=self.fov_h)
        self._kalman_trackers: dict = {}

        # Buffer de estados para evaluar estabilidad (sin limite fijo: purgamos por tiempo)
        self._state_history: deque = deque()
        self._latest_state: dict | None = None
        self._is_stable: bool = False

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
            PersonDetectionArray, '/detection/persons',
            self._detections_callback, qos_sensor,
        )
        self.sub_drone_state = self.create_subscription(
            DroneState, '/telemetry/drone/state',
            self._drone_state_callback, qos_sensor,
        )
        self.pub_persons_location = self.create_publisher(
            PersonLocationArray, '/gps/persons_location', qos_pub
        )

        self.frame_count = 0
        self.processed_count = 0
        self.rejected_count = 0

        self.get_logger().info('Georeferencing Node NRT listo. Esperando estabilidad del dron...')

    # ------------------------------------------------------------------
    # Telemetria
    # ------------------------------------------------------------------

    def _drone_state_callback(self, msg: DroneState) -> None:
        timestamp = msg.timestamp.sec + msg.timestamp.nanosec / 1e9
        state = {
            'timestamp':    timestamp,
            'latitude':     msg.latitude,
            'longitude':    msg.longitude,
            'altitude_rel': msg.altitude_rel,
            'altitude_abs': msg.altitude_abs,
            'yaw':          msg.yaw,
            'pitch':        msg.pitch,
            'roll':         msg.roll,
        }
        self._latest_state = state
        self._state_history.append(state)

        # Conservar el doble de la ventana para evitar perder estados en el borde
        cutoff = timestamp - self.stability_window_s * 2.0
        while self._state_history and self._state_history[0]['timestamp'] < cutoff:
            self._state_history.popleft()

        self._is_stable = self._check_stability(timestamp)

        if self._is_stable:
            self.get_logger().info('Dron estable — georeferenciacion activa.')
        else:
            window = timestamp - self._state_history[0]['timestamp'] if self._state_history else 0.0
            self.get_logger().info(
                f'Dron inestable (ventana={window:.1f}s / {self.stability_window_s:.1f}s requeridos).'
            )

    def _check_stability(self, now: float) -> bool:
        """True si la ventana cubre >= stability_window_s y todos los stds estan bajo umbral."""
        if len(self._state_history) < 2:
            return False

        oldest = self._state_history[0]['timestamp']
        if (now - oldest) < self.stability_window_s:
            return False

        lats = [s['latitude'] for s in self._state_history]
        lons = [s['longitude'] for s in self._state_history]
        yaws = [s['yaw'] for s in self._state_history]
        pitches = [s['pitch'] for s in self._state_history]

        ref_lat = lats[0]
        lon_m_per_deg = _LAT_M_PER_DEG * math.cos(math.radians(ref_lat))

        std_lat_m = float(np.std(lats)) * _LAT_M_PER_DEG
        std_lon_m = float(np.std(lons)) * lon_m_per_deg
        std_yaw = float(np.std(yaws))
        std_pitch = float(np.std(pitches))

        pos_ok = max(std_lat_m, std_lon_m) <= self.stability_pos_std_m
        angle_ok = max(std_yaw, std_pitch) <= self.stability_angle_std_deg

        self.get_logger().info(
            f'Estabilidad: pos={max(std_lat_m,std_lon_m):.2f}m '
            f'ang={max(std_yaw,std_pitch):.2f}deg '
            f'pos_ok={pos_ok} ang_ok={angle_ok}'
        )

        return pos_ok and angle_ok

    # ------------------------------------------------------------------
    # Detecciones
    # ------------------------------------------------------------------

    def _detections_callback(self, msg: PersonDetectionArray) -> None:
        self.frame_count += 1

        if not self._is_stable:
            return

        if self._latest_state is None:
            return

        drone_state = self._latest_state
        detection_time = msg.timestamp.sec + msg.timestamp.nanosec / 1e9

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
                continue

            if not (self.min_distance <= gps_location.distance_m <= self.max_distance):
                self.rejected_count += 1
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

        if self.use_kalman:
            for tid in set(self._kalman_trackers.keys()) - current_ids:
                del self._kalman_trackers[tid]

        self.pub_persons_location.publish(location_array)

        if self.frame_count % 50 == 0:
            total = self.processed_count + self.rejected_count
            rate = (self.rejected_count / max(1, total)) * 100
            self.get_logger().info(
                f'Frame {self.frame_count} | '
                f'Personas: {len(location_array.persons)} | '
                f'Rechazo: {rate:.1f}%'
            )


def main(args=None):
    rclpy.init(args=args)
    node = GeoreferencingNodeNrt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

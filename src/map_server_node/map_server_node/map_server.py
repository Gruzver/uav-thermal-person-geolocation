#!/usr/bin/env python3

"""
map_server.py

Nodo ROS2 que agrega el estado del dron, las ubicaciones GPS de personas
detectadas y el video anotado, y los persiste en disco para que el servidor
web los consuma.

Suscripciones:
    /gps/persons_location           (drone_tracker_msgs/PersonLocationArray)
    /telemetry/drone/state          (drone_tracker_msgs/DroneState)
    /vision/labeled_image_compressed (sensor_msgs/CompressedImage)

Salidas en output_dir (/tmp/drone_map_data por defecto):
    map_data.json       Estado completo con personas y posicion del dron.
    statistics.json     Metricas agregadas de la sesion.
    current_frame.jpg   Ultimo frame anotado (escritura atomica).

La escritura de la imagen usa rename atomico para evitar que el servidor web
lea un archivo a mitad de escritura.
"""

import json
import threading
from collections import defaultdict, deque
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from drone_tracker_msgs.msg import DroneState, PersonLocationArray
from sensor_msgs.msg import CompressedImage
import numpy as np


class MapServerNode(Node):
    """
    Agrega datos del sistema de deteccion y los persiste en disco a una
    frecuencia configurable para consumo por el dashboard web.

    Parametros ROS2:
        output_dir          (str)   Directorio de salida (default: /tmp/drone_map_data).
        update_rate         (int)   Frecuencia de escritura de JSON en Hz (default: 30).
        keep_history        (bool)  Mantener historial de posiciones por persona (default: False).
        max_history_frames  (int)   Maximo de entradas en el historial por persona (default: 500).
    """

    # Archivos de salida
    _FILENAMES = {
        'map_data':    'map_data.json',
        'statistics':  'statistics.json',
        'frame_temp':  'current_frame_temp.jpg',
        'frame_final': 'current_frame.jpg',
    }

    def __init__(self):
        super().__init__('map_server_node')

        self.declare_parameter('output_dir', '/tmp/drone_map_data')
        self.declare_parameter('update_rate', 30)
        self.declare_parameter('keep_history', False)
        self.declare_parameter('max_history_frames', 500)

        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.update_rate = self.get_parameter('update_rate').value
        self.keep_history = self.get_parameter('keep_history').value
        self.max_history_frames = self.get_parameter('max_history_frames').value

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._clean_stale_files()

        self.get_logger().info(f'Output dir:    {self.output_dir}')
        self.get_logger().info(f'Keep history:  {self.keep_history}')
        self.get_logger().info(f'Update rate:   {self.update_rate} Hz')

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_persons_location = self.create_subscription(
            PersonLocationArray,
            '/gps/persons_location',
            self._persons_location_callback,
            qos_sensor,
        )
        self.sub_drone_state = self.create_subscription(
            DroneState,
            '/telemetry/drone/state',
            self._drone_state_callback,
            qos_sensor,
        )
        self.sub_labeled_image = self.create_subscription(
            CompressedImage,
            '/vision/labeled_image_compressed',
            self._labeled_image_callback,
            qos_sensor,
        )

        # Estado compartido entre callbacks y el timer de escritura
        self.drone_state = None
        # {track_id: dict con metadatos y posicion actual}
        self.persons_data: dict = defaultdict(dict)
        self.frame_count = 0

        # Lock para acceso thread-safe al estado compartido
        self._data_lock = threading.Lock()

        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info('Map Server Node listo.')

    # ------------------------------------------------------------------
    # Inicializacion
    # ------------------------------------------------------------------

    def _clean_stale_files(self) -> None:
        """Elimina archivos de datos de sesiones anteriores al arrancar."""
        stale = [
            self.output_dir / self._FILENAMES['map_data'],
            self.output_dir / self._FILENAMES['statistics'],
            self.output_dir / self._FILENAMES['frame_final'],
        ]
        for path in stale:
            if path.exists():
                path.unlink()
                self.get_logger().info(f'Archivo previo eliminado: {path.name}')

    # ------------------------------------------------------------------
    # Callbacks de suscripcion
    # ------------------------------------------------------------------

    def _drone_state_callback(self, msg: DroneState) -> None:
        """Actualiza el estado del dron en memoria."""
        with self._data_lock:
            self.drone_state = {
                'latitude':     msg.latitude,
                'longitude':    msg.longitude,
                'altitude_abs': msg.altitude_abs,
                'altitude_rel': msg.altitude_rel,
                'yaw':          msg.yaw,
                'pitch':        msg.pitch,
                'roll':         msg.roll,
                'focal_len':    msg.focal_len,
                'dzoom_ratio':  msg.dzoom_ratio,
                'timestamp':    self._ros_time_to_iso(msg.timestamp),
            }

    def _persons_location_callback(self, msg: PersonLocationArray) -> None:
        """
        Actualiza el ciclo de vida de cada persona detectada.

        Las personas presentes en el mensaje se marcan como activas. Las que
        estaban activas y ya no aparecen en el frame actual se marcan como
        perdidas.

        Args:
            msg: Array con las ubicaciones GPS de personas en este frame.
        """
        with self._data_lock:
            self.frame_count += 1
            current_ids = set()

            for person in msg.persons:
                track_id = person.track_id
                current_ids.add(track_id)

                if track_id not in self.persons_data:
                    history: deque = deque(maxlen=self.max_history_frames)
                    self.persons_data[track_id] = {
                        'track_id':          track_id,
                        'first_seen_frame':  self.frame_count,
                        'detections_count':  0,
                        'history':           history,
                        'current':           None,
                        'last_known_location': None,
                        'status':            'active',
                    }

                current_data = {
                    'latitude':   person.latitude,
                    'longitude':  person.longitude,
                    'distance_m': person.distance_m,
                    'confidence': person.confidence,
                    'frame':      self.frame_count,
                    'timestamp':  self._ros_time_to_iso(person.timestamp),
                }

                entry = self.persons_data[track_id]
                entry['current'] = current_data
                entry['last_seen_frame'] = self.frame_count
                entry['last_known_location'] = {
                    'latitude':  person.latitude,
                    'longitude': person.longitude,
                    'frame':     self.frame_count,
                }
                entry['status'] = 'active'
                entry['detections_count'] += 1

                if self.keep_history:
                    # deque con maxlen descarta automaticamente las entradas mas antiguas
                    entry['history'].append(current_data)

            # Marcar como perdidas las personas que no aparecieron en este frame
            for track_id, entry in self.persons_data.items():
                if track_id not in current_ids and entry['status'] == 'active':
                    entry['status'] = 'lost'

    def _labeled_image_callback(self, msg: CompressedImage) -> None:
        """
        Decodifica el frame anotado y lo guarda en disco con escritura atomica.

        Usa un archivo temporal y rename para evitar que el servidor web lea
        un JPEG a mitad de escritura.

        Args:
            msg: Imagen comprimida en JPEG con las anotaciones de deteccion.
        """
        try:
            frame = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)

            if frame is None or frame.size == 0:
                self.get_logger().warning('Frame vacio recibido. Imagen descartada.')
                return

            temp_path = self.output_dir / self._FILENAMES['frame_temp']
            final_path = self.output_dir / self._FILENAMES['frame_final']

            success = cv2.imwrite(
                str(temp_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
            )
            if not success:
                self.get_logger().error('Error escribiendo imagen temporal en disco.')
                return

            # Rename atomico: el servidor web nunca ve un JPEG a mitad de escritura
            temp_path.replace(final_path)

        except Exception as e:
            self.get_logger().error(f'Error guardando frame: {e}')

    # ------------------------------------------------------------------
    # Escritura periodica de JSON
    # ------------------------------------------------------------------

    def _timer_callback(self) -> None:
        """
        Serializa el estado actual a JSON y lo escribe en disco.

        Se ejecuta a la frecuencia definida por update_rate. Si aun no hay
        datos del dron disponibles, omite la escritura.
        """
        with self._data_lock:
            if self.drone_state is None:
                return

            active_count = sum(
                1 for p in self.persons_data.values() if p['status'] == 'active'
            )
            lost_count = sum(
                1 for p in self.persons_data.values() if p['status'] == 'lost'
            )

            map_data = {
                'timestamp':    datetime.now().isoformat(),
                'frame_number': self.frame_count,
                'dron':         self.drone_state,
                'persons':      {},
                'statistics': {
                    'total_persons_detected': len(self.persons_data),
                    'active_persons':         active_count,
                    'lost_persons':           lost_count,
                    'unique_ids':             len(self.persons_data),
                    'total_frames_processed': self.frame_count,
                },
            }

            for track_id, entry in self.persons_data.items():
                person_json = {
                    'track_id':            entry['track_id'],
                    'status':              entry['status'],
                    'first_seen_frame':    entry['first_seen_frame'],
                    'last_seen_frame':     entry.get('last_seen_frame', 0),
                    'detections_count':    entry['detections_count'],
                    'current':             entry['current'],
                    'last_known_location': entry['last_known_location'],
                }
                if self.keep_history and entry['history']:
                    # Limitar a los ultimos 50 en el JSON para no saturarlo
                    person_json['history'] = list(entry['history'])[-50:]

                map_data['persons'][str(track_id)] = person_json

            self._write_json(self.output_dir / self._FILENAMES['map_data'], map_data)

            stats = {
                'timestamp':        datetime.now().isoformat(),
                'frame_count':      self.frame_count,
                'active_persons':   active_count,
                'lost_persons':     lost_count,
                'unique_persons':   len(self.persons_data),
                'total_detections': sum(
                    p['detections_count'] for p in self.persons_data.values()
                ),
            }
            self._write_json(self.output_dir / self._FILENAMES['statistics'], stats)

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------

    def _write_json(self, path: Path, data: dict) -> None:
        """
        Serializa data a JSON y lo escribe en path.

        Args:
            path: Ruta destino del archivo JSON.
            data: Diccionario a serializar.
        """
        try:
            with open(path, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Error escribiendo {path.name}: {e}')

    @staticmethod
    def _ros_time_to_iso(ros_time) -> str:
        """
        Convierte un timestamp ROS2 (sec + nanosec) a formato ISO 8601.

        Args:
            ros_time: Objeto con campos .sec y .nanosec.

        Returns:
            String en formato ISO 8601, por ejemplo '2026-03-30T14:23:01.123456'.
        """
        dt = datetime.fromtimestamp(ros_time.sec + ros_time.nanosec / 1e9)
        return dt.isoformat()


def main(args=None):
    rclpy.init(args=args)
    node = MapServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

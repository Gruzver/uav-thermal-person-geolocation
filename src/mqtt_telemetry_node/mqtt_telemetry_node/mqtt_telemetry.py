#!/usr/bin/env python3

"""
mqtt_telemetry.py

Nodo ROS2 que se suscribe al broker MQTT EMQX y traduce el payload OSD del
Mavic 3T al mensaje DroneState de ROS2.

El dron publica datos OSD cada ~2 segundos en el topico:
    thing/product/{SN}/osd

Estructura del payload relevante:
    {
        "timestamp": <unix_ms>,
        "data": {
            "latitude": float,
            "longitude": float,
            "height": float,       # altitud absoluta (m)
            "elevation": float,    # altitud relativa al punto de despegue (m)
            "67-0-0": {
                "gimbal_yaw":   float,
                "gimbal_pitch": float,
                "gimbal_roll":  float,
                "zoom_factor":  float,
            },
            ...
        }
    }

Publicaciones:
    /telemetry/drone/state  (drone_tracker_msgs/DroneState)

Reemplaza exactamente lo que el SRTParser + video_publisher proveian como
telemetria en el pipeline offline.
"""

import json
import os
import threading

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from drone_tracker_msgs.msg import DroneState

# Indice del payload de la camara termica en el OSD del Mavic 3T
_THERMAL_PAYLOAD_KEY = '67-0-0'

# Focal length equivalente de la camara termica del Mavic 3T (en mm)
# Se puede sobreescribir con el parametro ROS2 'focal_len'
_DEFAULT_FOCAL_LEN = 40.0


class MQTTTelemetryNode(Node):
    """
    Puente MQTT -> ROS2 para telemetria del dron Mavic 3T.

    El cliente MQTT corre en un hilo de fondo (loop_start). El callback
    on_message publica directamente en el topico ROS2; los publishers de
    rclpy son thread-safe.

    Las credenciales y el numero de serie identifican una aeronave y un broker
    concretos, por lo que no tienen valor por defecto. Se pasan como parametros
    ROS2 o, si se dejan vacios, se leen de las variables de entorno
    DJI_MQTT_USERNAME, DJI_MQTT_PASSWORD y DJI_AIRCRAFT_SN.

    Parametros ROS2:
        broker_host   (str)   Host del broker MQTT (default: "127.0.0.1").
        broker_port   (int)   Puerto del broker (default: 1883).
        username      (str)   Usuario MQTT. Sin default.
        password      (str)   Contrasena MQTT. Sin default.
        aircraft_sn   (str)   Numero de serie de la aeronave. Sin default.
        focal_len     (float) Focal length de la camara termica en mm (default: 40.0).
    """

    def __init__(self):
        super().__init__('mqtt_telemetry_node')

        self.declare_parameter('broker_host', '127.0.0.1')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('username', '')
        self.declare_parameter('password', '')
        self.declare_parameter('aircraft_sn', '')
        self.declare_parameter('focal_len', _DEFAULT_FOCAL_LEN)

        self.broker_host = self.get_parameter('broker_host').value
        self.broker_port = self.get_parameter('broker_port').value
        self.username = self.get_parameter('username').value or os.environ.get('DJI_MQTT_USERNAME', '')
        self.password = self.get_parameter('password').value or os.environ.get('DJI_MQTT_PASSWORD', '')
        self.aircraft_sn = self.get_parameter('aircraft_sn').value or os.environ.get('DJI_AIRCRAFT_SN', '')
        self.focal_len = self.get_parameter('focal_len').value

        if not self.aircraft_sn:
            raise ValueError(
                'aircraft_sn no configurado. Indicalo como parametro ROS2 '
                '(aircraft_sn:=<serie>) o via la variable de entorno DJI_AIRCRAFT_SN.'
            )

        self._topic = f'thing/product/{self.aircraft_sn}/osd'

        self.get_logger().info(f'Broker: {self.broker_host}:{self.broker_port}')
        self.get_logger().info(f'Topico MQTT: {self._topic}')
        self.get_logger().info(f'Focal length: {self.focal_len} mm')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub_drone_state = self.create_publisher(
            DroneState, '/telemetry/drone/state', qos
        )

        self._msg_count = 0
        self._lock = threading.Lock()

        self._mqtt_client = mqtt.Client(
            client_id=f'ros2_telemetry_{self.aircraft_sn}'
        )
        self._mqtt_client.username_pw_set(self.username, self.password)
        self._mqtt_client.on_connect = self._on_connect
        self._mqtt_client.on_disconnect = self._on_disconnect
        self._mqtt_client.on_message = self._on_message

        # Reconexion automatica con backoff exponencial (1s -> 30s)
        self._mqtt_client.reconnect_delay_set(min_delay=1, max_delay=30)

        try:
            self._mqtt_client.connect(self.broker_host, self.broker_port, keepalive=60)
        except Exception as e:
            self.get_logger().error(
                f'No se pudo conectar al broker MQTT: {e}. '
                'El nodo intentara reconectarse automaticamente.'
            )

        # Iniciar el loop MQTT en hilo de fondo
        self._mqtt_client.loop_start()

        self.get_logger().info('MQTT Telemetry Node listo. Esperando mensajes OSD...')

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f'Conectado al broker MQTT. Suscrito a: {self._topic}')
            client.subscribe(self._topic, qos=1)
        else:
            self.get_logger().error(f'Error de conexion MQTT. Codigo: {rc}')

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warning(
                f'Desconectado del broker MQTT (codigo: {rc}). Reconectando...'
            )

    def _on_message(self, client, userdata, message):
        """
        Callback MQTT: parsea el payload OSD y publica DroneState.

        Corre en el hilo del loop MQTT. Los publishers rclpy son thread-safe,
        por lo que se puede publicar directamente desde aqui.
        """
        try:
            payload = json.loads(message.payload.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            self.get_logger().error(f'Error decodificando payload MQTT: {e}')
            return

        data = payload.get('data', {})
        ts_ms = payload.get('timestamp')

        latitude = data.get('latitude')
        longitude = data.get('longitude')

        if latitude is None or longitude is None:
            self.get_logger().warning(
                'Payload OSD sin latitude/longitude. Mensaje descartado.'
            )
            return

        thermal = data.get(_THERMAL_PAYLOAD_KEY, {})
        if not thermal:
            self.get_logger().warning(
                f'Payload OSD sin datos de camara termica ({_THERMAL_PAYLOAD_KEY}). '
                'Se usan angulos de gimbal = 0.'
            )

        drone_state = DroneState()

        # Usar el reloj ROS al momento de recibir el mensaje MQTT.
        # El campo timestamp del dron DJI usa el reloj interno del dron, que
        # no esta sincronizado con el reloj de la laptop. Si se usa ese valor,
        # el georeferenciador no puede hacer match con los timestamps de las
        # imagenes (que vienen del reloj ROS). La latencia MQTT es <200ms,
        # despreciable frente al intervalo de 2s entre mensajes OSD.
        drone_state.timestamp = self.get_clock().now().to_msg()

        drone_state.latitude = float(latitude)
        drone_state.longitude = float(longitude)
        drone_state.altitude_abs = float(data.get('height', 0.0))
        drone_state.altitude_rel = float(data.get('elevation', 0.0))
        drone_state.yaw = float(thermal.get('gimbal_yaw', 0.0))
        drone_state.pitch = float(thermal.get('gimbal_pitch', 0.0))
        drone_state.roll = float(thermal.get('gimbal_roll', 0.0))
        drone_state.focal_len = float(self.focal_len)
        drone_state.dzoom_ratio = float(thermal.get('zoom_factor', 1.0))

        self.pub_drone_state.publish(drone_state)

        with self._lock:
            self._msg_count += 1
            count = self._msg_count

        if count % 10 == 0 or count <= 3:
            self.get_logger().info(
                f'[#{count}] GPS: ({drone_state.latitude:.6f}, {drone_state.longitude:.6f}) '
                f'alt_rel: {drone_state.altitude_rel:.1f} m  '
                f'gimbal yaw/pitch/roll: {drone_state.yaw:.1f}/{drone_state.pitch:.1f}/{drone_state.roll:.1f}'
            )

    def destroy_node(self):
        self._mqtt_client.loop_stop()
        self._mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MQTTTelemetryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Detenido por el usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import os
import threading
import urllib.request
import json
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage

# Opciones ffmpeg para minimizar latencia: sin buffer, modo low-delay, UDP
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = (
    'rtsp_transport;udp|fflags;nobuffer|flags;low_delay|max_delay;0|analyzeduration;0|probesize;32'
)


class RtspPublisherNode(Node):
    """
    Captura un stream RTSP (del drone via MediaMTX) y lo publica como topicos ROS2.

    Un hilo de fondo lee frames continuamente y descarta los viejos, de modo que
    el timer siempre publica el frame mas reciente (baja latencia).

    Si rtsp_url esta vacio, consulta la API de MediaMTX para auto-descubrir
    el primer stream activo bajo /live/.

    Parametros ROS2:
        rtsp_url        (str)  URL RTSP completa. Si vacio, auto-descubre.
        mediamtx_api    (str)  URL base de la API de MediaMTX.
        publish_rate    (int)  Frecuencia de publicacion en Hz (default: 30).
        use_compression (bool) Publicar imagen JPEG comprimida (default: True).
    """

    CAMERA_FRAME_ID = 'camera_thermal_optical'

    def __init__(self):
        super().__init__('rtsp_publisher_node')

        self.declare_parameter('rtsp_url', '')
        self.declare_parameter('mediamtx_api', 'http://127.0.0.1:9997')
        self.declare_parameter('publish_rate', 30)
        self.declare_parameter('use_compression', True)

        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.mediamtx_api = self.get_parameter('mediamtx_api').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_compression = self.get_parameter('use_compression').value

        if not self.rtsp_url:
            self.rtsp_url = self._discover_stream()

        if not self.rtsp_url:
            raise RuntimeError(
                'No se encontro ningun stream activo en MediaMTX. '
                'Asegurate de que el dron esta transmitiendo, o pasa rtsp_url manualmente.'
            )

        self.get_logger().info(f'Stream: {self.rtsp_url}')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_raw = self.create_publisher(Image, '/camera/thermal/image_raw', qos)
        self.pub_compressed = self.create_publisher(
            CompressedImage, '/camera/thermal/image_compressed', qos
        )

        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self._running = True
        self.frame_count = 0

        self._cap_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._cap_thread.start()

        self.timer = self.create_timer(1.0 / self.publish_rate, self._timer_callback)
        self.get_logger().info('RTSP Publisher listo.')

    def _discover_stream(self):
        try:
            url = f'{self.mediamtx_api}/v3/paths/list'
            self.get_logger().info(f'Buscando streams en {url} ...')
            with urllib.request.urlopen(url, timeout=5) as resp:
                data = json.loads(resp.read())
            for path in data.get('items', []):
                name = path.get('name', '')
                if path.get('ready') and path.get('tracks') and name.startswith('live/'):
                    rtsp = self.mediamtx_api.replace(':9997', ':8554').replace('http://', 'rtsp://')
                    found = f'{rtsp}/{name}'
                    self.get_logger().info(f'Stream encontrado: {found}')
                    return found
            self.get_logger().warning('No hay streams activos en MediaMTX.')
        except Exception as e:
            self.get_logger().error(f'Error consultando API de MediaMTX: {e}')
        return ''

    def _open_capture(self):
        cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _capture_loop(self):
        """Hilo que lee frames continuamente y siempre guarda solo el ultimo."""
        cap = self._open_capture()
        errors = 0

        while self._running:
            if not cap.isOpened():
                self.get_logger().warning('Reconectando al stream...')
                cap.release()
                cap = self._open_capture()
                errors = 0
                continue

            ret, frame = cap.read()
            if not ret:
                errors += 1
                if errors > 60:
                    cap.release()
                continue

            errors = 0
            with self._frame_lock:
                self._latest_frame = frame

        cap.release()

    def _to_image_msg(self, frame, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.CAMERA_FRAME_ID
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * frame.shape[2]
        msg.data = frame.tobytes()
        return msg

    def _timer_callback(self):
        with self._frame_lock:
            frame = self._latest_frame

        if frame is None:
            return

        self.frame_count += 1
        stamp = self.get_clock().now().to_msg()

        self.pub_raw.publish(self._to_image_msg(frame, stamp))

        if self.use_compression:
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                msg = CompressedImage()
                msg.header.stamp = stamp
                msg.header.frame_id = self.CAMERA_FRAME_ID
                msg.format = 'jpeg'
                msg.data = buf.tobytes()
                self.pub_compressed.publish(msg)

        if self.frame_count % 150 == 0:
            self.get_logger().info(f'Frames publicados: {self.frame_count}')


def main(args=None):
    rclpy.init(args=args)
    node = RtspPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Detenido por el usuario.')
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

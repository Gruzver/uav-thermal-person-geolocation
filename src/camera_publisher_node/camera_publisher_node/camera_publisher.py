#!/usr/bin/env python3

"""
camera_publisher.py

Nodo ROS2 que captura video en tiempo real desde una capturadora USB (HDMI->USB)
conectada al mando DJI y publica los frames como topicos de imagen.

Publicaciones:
    /camera/thermal/image_raw        (sensor_msgs/Image, bgr8)
    /camera/thermal/image_compressed (sensor_msgs/CompressedImage, JPEG)

Es el reemplazo en tiempo real de video_publisher_node para el pipeline live.
Los topicos publicados son identicos, por lo que el resto del pipeline
(YOLO, georeferenciador, map server) no requiere modificacion.
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class CameraPublisherNode(Node):
    """
    Captura video en vivo y lo publica como topicos ROS2.

    Parametros ROS2:
        device_path     (str)  Ruta V4L2 del dispositivo (default: /dev/video3).
        device_index    (int)  Indice usado si device_path esta vacio (default: 3).
        frame_width     (int)  Ancho de captura solicitado (default: 1920).
        frame_height    (int)  Alto de captura solicitado (default: 1080).
        fps             (int)  FPS solicitado al dispositivo (default: 30).
        publish_rate    (int)  Frecuencia de publicacion en Hz (default: 30).
        pixel_format    (str)  FourCC solicitado al dispositivo (default: MJPG).
        use_compression (bool) Publicar tambien imagen JPEG comprimida (default: True).
    """

    CAMERA_FRAME_ID = 'camera_thermal_optical'

    def __init__(self):
        super().__init__('camera_publisher_node')

        self.declare_parameter('device_path', '/dev/video3')
        self.declare_parameter('device_index', 3)
        self.declare_parameter('frame_width', 1920)
        self.declare_parameter('frame_height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_rate', 30)
        self.declare_parameter('pixel_format', 'MJPG')
        self.declare_parameter('use_compression', True)

        self.device_path = self.get_parameter('device_path').value
        self.device_index = self.get_parameter('device_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.pixel_format = self.get_parameter('pixel_format').value
        self.use_compression = self.get_parameter('use_compression').value

        self.device = self.device_path or self.device_index

        self.get_logger().info(f'Dispositivo: {self.device}')
        self.get_logger().info(f'Resolucion solicitada: {self.frame_width}x{self.frame_height}')
        self.get_logger().info(f'Formato solicitado: {self.pixel_format} @ {self.fps} FPS')
        self.get_logger().info(f'Frecuencia de publicacion: {self.publish_rate} Hz')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_image_raw = self.create_publisher(
            Image, '/camera/thermal/image_raw', qos
        )
        self.pub_image_compressed = self.create_publisher(
            CompressedImage, '/camera/thermal/image_compressed', qos
        )

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(
                f'No se pudo abrir el dispositivo de video {self.device}. '
                'Verifica que la capturadora este conectada.'
            )

        if self.pixel_format:
            fourcc = cv2.VideoWriter_fourcc(*self.pixel_format[:4])
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Reportar la configuracion que el dispositivo realmente entrega.
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        actual_format = ''.join(chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4))

        self.get_logger().info(
            f'Captura efectiva: {actual_w}x{actual_h} {actual_format} @ {actual_fps:.2f} FPS'
        )

        self.frame_count = 0
        self._read_errors = 0

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info('Camera Publisher listo. Publicando en tiempo real.')

    def _timer_callback(self) -> None:
        ret, frame = self.cap.read()

        if not ret:
            self._read_errors += 1
            if self._read_errors == 1 or self._read_errors % 30 == 0:
                self.get_logger().warning(
                    f'No se pudo leer frame de la capturadora '
                    f'(error #{self._read_errors}). Verifica la conexion.'
                )
            return

        self._read_errors = 0
        self.frame_count += 1
        current_time = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = current_time
        img_msg.header.frame_id = self.CAMERA_FRAME_ID
        self.pub_image_raw.publish(img_msg)

        if self.use_compression:
            ok, buffer = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
            )
            if ok:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = current_time
                compressed_msg.header.frame_id = self.CAMERA_FRAME_ID
                compressed_msg.format = 'jpeg'
                compressed_msg.data = buffer.tobytes()
                self.pub_image_compressed.publish(compressed_msg)
            else:
                self.get_logger().warning(
                    f'Frame {self.frame_count}: fallo la compresion JPEG.'
                )

        if self.frame_count % 150 == 0:
            self.get_logger().info(f'Frames publicados: {self.frame_count}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Detenido por el usuario.')
    finally:
        if hasattr(node, 'cap') and node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

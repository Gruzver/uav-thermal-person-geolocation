#!/usr/bin/env python3

"""
yolo_detector.py

Nodo ROS2 que recibe frames de la camara termica, ejecuta inferencia con un
modelo YOLO con transfer learning y publica las detecciones con tracking
persistente de personas.

Suscripciones:
    /camera/thermal/image_compressed (sensor_msgs/CompressedImage, JPEG)

    Se usa el topico comprimido y no el raw (~3.9MB por frame a 1280x1024)
    porque publicar/recibir mensajes tan grandes por DDS resulto ser el
    cuello de botella real del pipeline (~150ms por mensaje, medido en este
    sistema, independiente del RMW usado), muy por encima del costo real de
    inferencia (~40ms). El JPEG a calidad 85 es 30-80x mas chico y el decode
    agrega solo unos pocos ms.

Publicaciones:
    /detection/persons              (drone_tracker_msgs/PersonDetectionArray)
    /vision/labeled_image_raw       (sensor_msgs/Image)
    /vision/labeled_image_compressed (sensor_msgs/CompressedImage)

Las coordenadas de bounding box en PersonDetection se reportan en el espacio
de inferencia YOLO (INFERENCE_W x INFERENCE_H) para que el nodo de
georeferenciacion las procese directamente sin necesidad de reescalar.
"""

import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from ultralytics import YOLO

from drone_tracker_msgs.msg import PersonDetection, PersonDetectionArray


# Resolucion de entrenamiento del modelo YOLO. Las detecciones se reportan
# en este espacio de coordenadas para mantener coherencia con GeoCalculator.
INFERENCE_W = 640
INFERENCE_H = 512


class YOLODetectionNode(Node):
    """
    Nodo de deteccion y tracking de personas con YOLO.

    Redimensiona cada frame entrante a la resolucion de entrenamiento antes
    de la inferencia. Las bounding boxes publicadas estan en el espacio
    INFERENCE_W x INFERENCE_H. La imagen anotada se publica en la resolucion
    original para mayor detalle visual.

    Parametros ROS2:
        model_path        (str)   Ruta al modelo YOLO (.pt).
        conf_threshold    (float) Umbral de confianza minimo (default: 0.5).
        iou_threshold     (float) Umbral IoU para NMS (default: 0.45).
        use_half          (bool)  Habilitar inferencia en FP16 (default: False).
        device            (int)   Indice de GPU (default: 0). Usar -1 para CPU.
        tracker_config    (str)   Config de tracker de Ultralytics (default: 'bytetrack.yaml').
        max_frame_age_ms  (float) Edad maxima de un frame antes de descartarlo.
                                  Si la inferencia es mas lenta que el ritmo de
                                  publicacion, los frames acumulados en cola se
                                  descartan para evitar lag creciente (default: 100.0).
        image_qos_depth   (int)   Profundidad del QoS de la suscripcion de imagen
                                  (default: 1). En modo vivo, 1 es correcto (siempre
                                  el frame mas reciente). En batch offline, subir
                                  este valor para no perder frames intermedios por
                                  overflow del buffer cuando la inferencia es mas
                                  lenta que el ritmo de publicacion.
    """

    def __init__(self):
        super().__init__('yolo_detection_node')

        # Ruta a los pesos YOLOv8. Sin valor por defecto: el modelo entrenado se
        # distribuye aparte (ver README) y no forma parte de este repositorio.
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('use_half', False)
        self.declare_parameter('device', 0)
        self.declare_parameter('tracker_config', 'bytetrack.yaml')
        self.declare_parameter('max_frame_age_ms', 100.0)
        self.declare_parameter('image_qos_depth', 1)

        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.use_half = self.get_parameter('use_half').value
        self.device = self.get_parameter('device').value
        self.tracker_config = self.get_parameter('tracker_config').value
        self._max_frame_age_ms = self.get_parameter('max_frame_age_ms').value
        self.image_qos_depth = self.get_parameter('image_qos_depth').value

        if not self.model_path:
            raise ValueError(
                'model_path es obligatorio. Descarga los pesos entrenados (ver '
                'README) e indicalos al lanzar: model_path:=/ruta/a/best.pt'
            )

        self.get_logger().info(f'Modelo: {self.model_path}')
        self.get_logger().info(
            f'Conf: {self.conf_threshold:.2f}  IoU: {self.iou_threshold:.2f}'
            f'  FP16: {self.use_half}  Device: {self.device}'
        )
        self.get_logger().info(f'Max frame age: {self._max_frame_age_ms:.0f} ms')

        # QoS sensor: BEST_EFFORT para compatibilidad con camera_publisher.
        # depth=1 (default) es lo correcto en modo vivo: siempre se quiere el
        # frame mas reciente y descartar atraso acumulado. Para batch offline
        # esto rompe la continuidad del tracker (frames intermedios se pierden
        # a nivel de middleware, sin pasar por el chequeo de max_frame_age_ms),
        # por eso el launch offline sube image_qos_depth para cubrir el clip
        # completo sin perder frames.
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.image_qos_depth,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub_image_compressed = self.create_subscription(
            CompressedImage,
            '/camera/thermal/image_compressed',
            self._image_callback,
            qos_sensor,
        )
        self.pub_detections = self.create_publisher(
            PersonDetectionArray, '/detection/persons', qos_pub
        )
        self.pub_labeled_image_raw = self.create_publisher(
            Image, '/vision/labeled_image_raw', qos_pub
        )
        self.pub_labeled_image_compressed = self.create_publisher(
            CompressedImage, '/vision/labeled_image_compressed', qos_pub
        )

        # Cargar modelo YOLO
        try:
            self.get_logger().info('Cargando modelo YOLO...')
            self.model = YOLO(self.model_path)
            precision = 'FP16' if self.use_half else 'FP32'
            self.get_logger().info(f'Modelo cargado ({precision}).')
        except Exception as e:
            self.get_logger().error(f'Error cargando modelo: {e}')
            raise

        self.frame_count = 0
        self._dropped_frames = 0
        self._untracked_boxes = 0
        # Deque de tamano fijo para calcular el promedio de los ultimos N frames
        self.inference_times: deque = deque(maxlen=50)

        self.get_logger().info('YOLO Detection Node listo.')

    def _image_callback(self, msg: CompressedImage) -> None:
        """
        Procesa un frame entrante: escala a la resolucion de inferencia,
        ejecuta YOLO con tracking, construye los mensajes de deteccion y
        publica resultados.

        Antes de procesar verifica la edad del frame. Si la inferencia es mas
        lenta que el ritmo de publicacion, los frames que llevan demasiado
        tiempo en cola se descartan para que el nodo siempre trabaje con
        frames recientes y no acumule lag.

        Args:
            msg: Imagen JPEG comprimida de la camara termica.
        """
        # Descartar frames demasiado viejos para evitar lag acumulado.
        # Se compara el timestamp de publicacion del frame contra el reloj actual.
        msg_time_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_s = self.get_clock().now().nanoseconds * 1e-9
        age_ms = (now_s - msg_time_s) * 1000.0

        if age_ms > self._max_frame_age_ms:
            self._dropped_frames += 1
            # Throttle: primer aviso y luego cada 50 descartes
            if self._dropped_frames == 1 or self._dropped_frames % 50 == 0:
                self.get_logger().warning(
                    f'Frame descartado: edad {age_ms:.0f} ms > limite {self._max_frame_age_ms:.0f} ms. '
                    f'Total descartados: {self._dropped_frames}. '
                    f'Considera reducir publish_rate o aumentar max_frame_age_ms.'
                )
            return

        try:
            jpeg_buffer = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            frame = cv2.imdecode(jpeg_buffer, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warning('No se pudo decodificar el JPEG recibido. Frame omitido.')
                return
            frame_h, frame_w = frame.shape[:2]
            self.frame_count += 1

            # Escalar a la resolucion de entrenamiento del modelo
            frame_scaled = cv2.resize(frame, (INFERENCE_W, INFERENCE_H))

            inference_start = time.time()
            results = self.model.track(
                frame_scaled,
                persist=True,
                tracker=self.tracker_config,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False,
                half=self.use_half,
            )
            inference_time = (time.time() - inference_start) * 1000
            self.inference_times.append(inference_time)

            detection_array = PersonDetectionArray()
            detection_array.timestamp = msg.header.stamp

            # La imagen anotada se dibuja sobre el frame original para
            # preservar la resolucion maxima en la visualizacion.
            labeled_frame = frame.copy()
            scale_x = frame_w / INFERENCE_W
            scale_y = frame_h / INFERENCE_H

            if results and results[0].boxes is not None:
                boxes = results[0].boxes
                track_ids = boxes.id

                for i, box in enumerate(boxes):
                    if track_ids is None:
                        # El tracker aun no asigno un ID persistente a esta caja
                        # (ocurre en frames iniciales o tras perder la pista por
                        # completo). Se descarta en vez de inventar un ID falso,
                        # ya que georeferencing_node agrupa su estado de Kalman
                        # por track_id.
                        self._untracked_boxes += 1
                        if self._untracked_boxes == 1 or self._untracked_boxes % 50 == 0:
                            self.get_logger().warning(
                                f'Deteccion sin track_id persistente, descartada. '
                                f'Total: {self._untracked_boxes}.'
                            )
                        continue
                    track_id = int(track_ids[i].item())

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    centroid_x = (x1 + x2) / 2.0
                    centroid_y = (y1 + y2) / 2.0
                    confidence = float(box.conf.item())

                    # Coordenadas en espacio INFERENCE_W x INFERENCE_H
                    person_det = PersonDetection()
                    person_det.track_id = track_id
                    person_det.confidence = confidence
                    person_det.bbox_x1 = float(x1)
                    person_det.bbox_y1 = float(y1)
                    person_det.bbox_x2 = float(x2)
                    person_det.bbox_y2 = float(y2)
                    person_det.centroid_x = float(centroid_x)
                    person_det.centroid_y = float(centroid_y)
                    person_det.timestamp = msg.header.stamp
                    detection_array.detections.append(person_det)

                    # Dibujar bbox escalada al frame original
                    x1_draw = int(x1 * scale_x)
                    y1_draw = int(y1 * scale_y)
                    x2_draw = int(x2 * scale_x)
                    y2_draw = int(y2 * scale_y)
                    cv2.rectangle(
                        labeled_frame,
                        (x1_draw, y1_draw),
                        (x2_draw, y2_draw),
                        (0, 255, 0),
                        2,
                    )
                    label = f'ID:{track_id} {confidence:.2f}'
                    cv2.putText(
                        labeled_frame, label,
                        (x1_draw, y1_draw - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
                    )

            # Overlay de informacion en el frame anotado
            overlay = (
                f'Frame: {self.frame_count} | '
                f'Personas: {len(detection_array.detections)} | '
                f'Inf: {inference_time:.1f} ms'
            )
            cv2.putText(
                labeled_frame, overlay, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
            )

            self.pub_detections.publish(detection_array)

            labeled_msg = Image()
            labeled_msg.header = msg.header
            labeled_msg.height = labeled_frame.shape[0]
            labeled_msg.width = labeled_frame.shape[1]
            labeled_msg.encoding = 'bgr8'
            labeled_msg.is_bigendian = False
            labeled_msg.step = labeled_frame.shape[1] * labeled_frame.shape[2]
            labeled_msg.data = labeled_frame.tobytes()
            self.pub_labeled_image_raw.publish(labeled_msg)

            ok, buffer = cv2.imencode(
                '.jpg', labeled_frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
            )
            if ok:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = buffer.tobytes()
                self.pub_labeled_image_compressed.publish(compressed_msg)

            if self.frame_count % 50 == 0:
                avg_inference = np.mean(self.inference_times)
                self.get_logger().info(
                    f'Frame {self.frame_count} | '
                    f'Personas: {len(detection_array.detections)} | '
                    f'Inf promedio: {avg_inference:.2f} ms | '
                    f'Descartados: {self._dropped_frames}'
                )

        except Exception as e:
            self.get_logger().error(f'Error procesando imagen en frame {self.frame_count}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Detenido por el usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

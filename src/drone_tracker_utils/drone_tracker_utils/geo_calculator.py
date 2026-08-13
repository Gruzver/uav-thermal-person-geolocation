"""
geo_calculator.py

Calcula la ubicacion GPS de personas detectadas a partir de coordenadas en
pixeles, la posicion del dron y los parametros de la camara.

La conversion pixel -> GPS sigue estos pasos:
  1. Normalizar coordenada de pixel al rango [-0.5, 0.5]
  2. Calcular el angulo horizontal/vertical que subtiende el pixel desde el FOV
  3. Combinar el angulo vertical con el pitch del dron para obtener el angulo
     total de inclinacion hacia el suelo
  4. Calcular la distancia horizontal al suelo con trigonometria
  5. Proyectar esa distancia en X/Y aplicando el yaw del dron
  6. Convertir el desplazamiento en metros a delta de latitud/longitud
"""

import math
import logging
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class PersonLocation:
    """Resultado de la conversion pixel -> GPS para una persona detectada."""
    latitude: float
    longitude: float
    distance_m: float
    delta_x: float
    delta_y: float
    altitude_assumed: float = 0.0


class GeoCalculator:
    """
    Convierte coordenadas de pixel (espacio de imagen 640x512) a coordenadas
    GPS absolutas, teniendo en cuenta la posicion, altitud y orientacion del dron.
    """

    # Especificaciones de la camara termica DJI (resolucion de inferencia YOLO)
    CAMERA_SPECS = {
        'frame_width': 640,
        'frame_height': 512,
        # FOV diagonal nominal de la camara (grados). Se usa para derivar el FOV horizontal.
        'dfov': 61,
        'focal_length_mm': 40,
    }

    def __init__(self, fov_horizontal: Optional[float] = None):
        """
        Inicializa el calculador con el FOV horizontal de la camara.

        Args:
            fov_horizontal: FOV horizontal en grados. Si no se indica, se deriva
                            del FOV diagonal definido en CAMERA_SPECS.
        """
        if fov_horizontal is None:
            self.fov_h = self._calculate_fov_from_dfov(self.CAMERA_SPECS['dfov'])
        else:
            self.fov_h = fov_horizontal

        self.fov_v = self._calculate_fov_v_from_h(self.fov_h)

    @staticmethod
    def _calculate_fov_from_dfov(dfov: float) -> float:
        """
        Deriva el FOV horizontal a partir del FOV diagonal.

        El factor 0.612 es una aproximacion empirica calibrada para la camara
        termica DJI con sensor 640x512. La formula exacta dependeria del tamano
        fisico del sensor, que no esta disponible en los metadatos SRT.

        Args:
            dfov: FOV diagonal en grados.

        Returns:
            FOV horizontal aproximado en grados.
        """
        return dfov * 0.612

    @staticmethod
    def _calculate_fov_v_from_h(fov_h: float) -> float:
        """
        Calcula el FOV vertical a partir del FOV horizontal usando la razon de
        aspecto del sensor (frame_height / frame_width).

        Args:
            fov_h: FOV horizontal en grados.

        Returns:
            FOV vertical en grados.
        """
        aspect = (
            GeoCalculator.CAMERA_SPECS['frame_height']
            / GeoCalculator.CAMERA_SPECS['frame_width']
        )
        return fov_h * aspect

    def pixel_to_gps(
        self,
        bbox_center_x: float,
        bbox_center_y: float,
        latitude_drone: float,
        longitude_drone: float,
        altitude_drone: float,
        yaw: float,
        pitch: float,
        roll: float = 0.0,
    ) -> Optional[PersonLocation]:
        """
        Convierte el centroide de una deteccion (en pixeles) a coordenadas GPS.

        Args:
            bbox_center_x: Coordenada X del centroide en pixeles (0 a frame_width).
            bbox_center_y: Coordenada Y del centroide en pixeles (0 a frame_height).
            latitude_drone: Latitud del dron en grados decimales.
            longitude_drone: Longitud del dron en grados decimales.
            altitude_drone: Altitud relativa del dron sobre el suelo en metros.
            yaw: Yaw del gimbal en grados (Norte = 0, horario positivo).
            pitch: Pitch del gimbal en grados (negativo = mirando hacia abajo).
            roll: Roll del gimbal en grados (sin efecto en la proyeccion actual).

        Returns:
            PersonLocation con la posicion GPS calculada, o None si el calculo
            no es valido (pixel fuera de rango, angulo degenerado, distancia
            irreal, etc.).
        """
        frame_w = self.CAMERA_SPECS['frame_width']
        frame_h = self.CAMERA_SPECS['frame_height']

        if bbox_center_x > frame_w or bbox_center_y > frame_h:
            logger.warning(
                "Pixel fuera de rango: (%.0f, %.0f). Esperado: 0-%d x 0-%d",
                bbox_center_x, bbox_center_y, frame_w, frame_h,
            )
            return None

        try:
            yaw_rad = math.radians(yaw)
            pitch_rad = math.radians(pitch)

            # Normalizar pixel al rango [-0.5, 0.5]
            x_norm = (bbox_center_x / frame_w) - 0.5
            y_norm = (bbox_center_y / frame_h) - 0.5

            # Angulo que subtiende el pixel dentro del FOV.
            # Convencion de signo de angle_v: y_norm crece hacia abajo en el
            # espacio de imagen, y con la camara inclinada hacia el suelo un
            # pixel en la mitad inferior del frame corresponde a una linea de
            # vision mas cercana al nadir, es decir un angulo total mas
            # pronunciado. Como pitch es negativo hacia abajo, angle_v debe
            # llevar el signo opuesto a y_norm para que sumarlo a pitch acerque
            # el angulo total al nadir.
            angle_h = x_norm * math.radians(self.fov_h)
            angle_v = -y_norm * math.radians(self.fov_v)

            # Angulo vertical total: pitch del dron + desplazamiento del pixel
            angle_vertical_total = pitch_rad + angle_v

            # Extension de la formulacion al cruzar el nadir puro. Cerca de
            # pitch = -90 grados, angle_v puede llevar el angulo total mas alla
            # de -90: el rayo de vision ha rotado pasando la vertical y apunta
            # hacia el lado opuesto del yaw. Como |tan(theta)| es simetrico
            # respecto de -90 grados, la magnitud de la distancia no cambia,
            # pero el azimut debe rotarse 180 grados para seguir el rumbo real
            # del rayo. Se refleja el angulo sobre la vertical y se acumula ese
            # giro en azimuth_flip, aplicado mas abajo junto al yaw.
            azimuth_flip = 0.0
            if angle_vertical_total < -math.pi / 2:
                angle_vertical_total = -math.pi - angle_vertical_total
                azimuth_flip = math.pi

            # Evitar division por cero y angulos que no alcanzan el suelo
            if abs(math.cos(angle_vertical_total)) < 0.001:
                return None

            tan_angle = math.tan(angle_vertical_total)
            if abs(tan_angle) < 0.001:
                return None

            # Distancia horizontal desde la proyeccion vertical del dron
            horizontal_distance = altitude_drone / abs(tan_angle)

            if horizontal_distance > 5000 or horizontal_distance < 0:
                return None

            # Proyectar la distancia en los ejes Norte/Este aplicando yaw
            angle_horizontal_total = yaw_rad + angle_h + azimuth_flip
            delta_x = horizontal_distance * math.sin(angle_horizontal_total)
            delta_y = horizontal_distance * math.cos(angle_horizontal_total)

            # Convertir metros a grados de latitud/longitud
            meters_per_degree_lat = 111_000
            meters_per_degree_lon = 111_000 * math.cos(math.radians(latitude_drone))

            latitude_person = latitude_drone + delta_y / meters_per_degree_lat
            longitude_person = longitude_drone + delta_x / meters_per_degree_lon

            distance_total = math.sqrt(delta_x ** 2 + delta_y ** 2)

            return PersonLocation(
                latitude=latitude_person,
                longitude=longitude_person,
                distance_m=distance_total,
                delta_x=delta_x,
                delta_y=delta_y,
            )

        except Exception:
            logger.exception(
                "Error inesperado en pixel_to_gps para pixel (%.1f, %.1f)",
                bbox_center_x, bbox_center_y,
            )
            return None

    def validate_location(
        self, location: Optional[PersonLocation], max_distance: float = 1000.0
    ) -> bool:
        """
        Valida que una ubicacion calculada sea fisicamente razonable.

        Args:
            location: Resultado de pixel_to_gps. Puede ser None.
            max_distance: Distancia maxima aceptable en metros desde el dron.

        Returns:
            True si la ubicacion es valida, False en caso contrario.
        """
        if location is None:
            return False
        return 0.0 <= location.distance_m <= max_distance

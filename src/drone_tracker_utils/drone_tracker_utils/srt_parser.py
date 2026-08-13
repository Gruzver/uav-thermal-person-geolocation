"""
srt_parser.py

Parsea archivos SRT generados por camaras DJI para extraer la telemetria del
dron frame a frame (GPS, altitud, angulos de gimbal, parametros de camara).

El formato SRT de DJI embebe metadatos en bloques de subtitulos, por ejemplo:

    1
    00:00:00.000 --> 00:00:00.033
    <font size="28">SrtCnt : 1, DiffTime : 33ms
    [latitude: -12.123456] [longitude: -77.123456]
    [rel_alt: 45.600 abs_alt: 175.600]
    [gb_yaw: 32.5 gb_pitch: -35.0 gb_roll: 0.0]
    [focal_len: 40] [dzoom_ratio: 1.000]
    </font>
"""

import re
import logging
from pathlib import Path
from dataclasses import dataclass, field
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class DroneFrame:
    """Telemetria del dron para un frame especifico del video."""

    frame_number: int
    timestamp: str
    timestamp_ms: float
    latitude: float
    longitude: float
    altitude_rel: float = 0.0
    altitude_abs: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    focal_len: float = 40.0
    dzoom_ratio: float = 1.0


class SRTParser:
    """
    Parser de archivos SRT de DJI.

    Extrae telemetria por frame: posicion GPS, altitud relativa y absoluta,
    angulos del gimbal (yaw, pitch, roll) y parametros opticos de la camara.

    Uso:
        parser = SRTParser('/ruta/al/archivo.SRT')
        frames = parser.parse()
        frame = parser.get_frame_by_timestamp(timestamp_ms=5000.0)
    """

    # Patrones de extraccion de cada campo telemetrico
    _PATTERNS: Dict[str, str] = {
        'latitude':     r'\[latitude:\s*([-\d.]+)\]',
        'longitude':    r'\[longitude:\s*([-\d.]+)\]',
        'altitude_rel': r'\[rel_alt:\s*([\d.]+)',
        'altitude_abs': r'abs_alt:\s*([\d.]+)\]',
        'yaw':          r'\[gb_yaw:\s*([-\d.]+)',
        'pitch':        r'gb_pitch:\s*([-\d.]+)',
        'roll':         r'gb_roll:\s*([-\d.]+)\]',
        'focal_len':    r'\[focal_len:\s*([\d.]+)\]',
        'dzoom_ratio':  r'\[dzoom_ratio:\s*([\d.]+)\]',
    }

    def __init__(self, srt_path: str):
        self.srt_path = Path(srt_path)
        self.frames: List[DroneFrame] = []
        self.frame_dict: Dict[int, DroneFrame] = {}

    def parse(self) -> List[DroneFrame]:
        """
        Lee y parsea el archivo SRT completo.

        Returns:
            Lista de DroneFrame ordenados segun aparecen en el archivo.

        Raises:
            FileNotFoundError: Si el archivo SRT no existe.
        """
        if not self.srt_path.exists():
            raise FileNotFoundError(f"Archivo SRT no encontrado: {self.srt_path}")

        content = self.srt_path.read_text(encoding='utf-8')

        # Separar bloques de subtitulo. DJI puede usar \n\n o \r\n\r\n segun
        # el sistema operativo en el que se genero el archivo.
        blocks = re.split(r'\r?\n\r?\n', content.strip())

        for block in blocks:
            frame = self._parse_frame(block)
            if frame is not None:
                self.frames.append(frame)
                self.frame_dict[frame.frame_number] = frame

        self._calculate_timestamps()
        logger.debug("Parsed %d frames from %s", len(self.frames), self.srt_path.name)
        return self.frames

    def _parse_frame(self, block: str) -> Optional[DroneFrame]:
        """
        Parsea un bloque de texto SRT y devuelve un DroneFrame.

        Args:
            block: Texto crudo de un bloque de subtitulo SRT.

        Returns:
            DroneFrame si el bloque contiene datos validos, None en caso contrario.
        """
        lines = block.strip().splitlines()
        if len(lines) < 2:
            return None

        try:
            frame_number = int(lines[0].strip())
            timestamp = lines[1].split(' --> ')[0].replace(',', '.')
            data_text = '\n'.join(lines[2:])
            data = self._extract_data(data_text)

            if data is None:
                return None

            return DroneFrame(
                frame_number=frame_number,
                timestamp=timestamp,
                timestamp_ms=0.0,
                **data,
            )

        except (ValueError, IndexError):
            logger.debug("Bloque SRT omitido por formato inesperado: %r", block[:80])
            return None

    def _extract_data(self, text: str) -> Optional[Dict]:
        """
        Extrae los campos telemetricos del contenido textual de un bloque SRT.

        Elimina etiquetas HTML (usadas por DJI para estilos de subtitulo) antes
        de aplicar los patrones de extraccion.

        Args:
            text: Contenido textual del bloque (lineas despues del timestamp).

        Returns:
            Diccionario con los campos extraidos, o None si faltan latitud o
            longitud (campos obligatorios).
        """
        # Eliminar etiquetas HTML de DJI (<font>, </font>, etc.)
        clean = re.sub(r'<[^>]+>', '', text)
        clean = ' '.join(clean.split())

        data: Dict = {}
        for key, pattern in self._PATTERNS.items():
            match = re.search(pattern, clean)
            if match:
                try:
                    data[key] = float(match.group(1))
                except ValueError:
                    logger.debug("No se pudo convertir el campo '%s' a float: %r", key, match.group(1))

        if 'latitude' not in data or 'longitude' not in data:
            return None

        return data

    def _calculate_timestamps(self) -> None:
        """
        Convierte el campo timestamp (HH:MM:SS.mmm) de cada frame a
        milisegundos absolutos y lo almacena en DroneFrame.timestamp_ms.
        """
        for frame in self.frames:
            try:
                parts = frame.timestamp.split(':')
                hours = int(parts[0])
                minutes = int(parts[1])
                seconds = float(parts[2])
                frame.timestamp_ms = (hours * 3600 + minutes * 60 + seconds) * 1000.0
            except (ValueError, IndexError):
                logger.warning(
                    "No se pudo parsear el timestamp '%s' en frame %d. Se asigna 0.",
                    frame.timestamp, frame.frame_number,
                )
                frame.timestamp_ms = 0.0

    def get_frame(self, frame_number: int) -> Optional[DroneFrame]:
        """
        Devuelve el frame por su numero de secuencia SRT.

        Args:
            frame_number: Numero de frame (comienza en 1 en archivos DJI).

        Returns:
            DroneFrame correspondiente, o None si no existe.
        """
        return self.frame_dict.get(frame_number)

    def get_frame_by_timestamp(
        self, timestamp_ms: float, max_delta_ms: float = 500.0
    ) -> Optional[DroneFrame]:
        """
        Devuelve el frame cuyo timestamp sea mas cercano al solicitado.

        Args:
            timestamp_ms: Timestamp objetivo en milisegundos.
            max_delta_ms: Diferencia maxima aceptable en milisegundos. Si el
                          frame mas cercano supera este umbral, devuelve None.

        Returns:
            DroneFrame mas cercano, o None si no hay frames o si la diferencia
            supera max_delta_ms.
        """
        if not self.frames:
            return None

        closest = min(self.frames, key=lambda f: abs(f.timestamp_ms - timestamp_ms))
        delta = abs(closest.timestamp_ms - timestamp_ms)

        if delta > max_delta_ms:
            logger.debug(
                "Frame mas cercano tiene delta %.0f ms (limite: %.0f ms). Devuelve None.",
                delta, max_delta_ms,
            )
            return None

        return closest

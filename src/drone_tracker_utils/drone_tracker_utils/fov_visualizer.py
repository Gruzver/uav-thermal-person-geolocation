"""
fov_visualizer.py

Herramienta de diagnostico para visualizar el campo de vision (FOV) de la
camara del dron sobre un mapa real (Folium/OpenStreetMap).

Permite navegar frame a frame por los datos del archivo SRT y comprobar que
la proyeccion del FOV sea geometricamente coherente (forma de rectangulo
rotado segun el yaw, tamano proporcional a la altitud y el pitch, etc.).

Uso:
    python3 fov_visualizer.py
    # o bien, con ruta explicita:
    python3 fov_visualizer.py /ruta/al/archivo.SRT
"""

import logging
import math
import sys
import threading
import webbrowser
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

import folium

from drone_tracker_utils import GeoCalculator, SRTParser

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)


class FOVMapVisualizer:
    """
    Genera mapas Folium interactivos mostrando el FOV de la camara para cada
    frame del archivo SRT, y los sirve via HTTP para navegacion en el browser.
    """

    OUTPUT_DIR = Path('/tmp/fov_maps')

    def __init__(self, srt_path: str):
        """
        Carga el archivo SRT e inicializa el visualizador.

        Args:
            srt_path: Ruta al archivo SRT de DJI.
        """
        parser = SRTParser(srt_path)
        self.frames = parser.parse()
        self.current_frame_idx = 0
        self.geo_calc = GeoCalculator(fov_horizontal=50.5)

        logger.info("Cargados %d frames del SRT.", len(self.frames))

        self.OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        self.update_map()
        self._start_server()

    # ------------------------------------------------------------------
    # Navegacion de frames
    # ------------------------------------------------------------------

    def _current_frame(self):
        """Devuelve el DroneFrame en el indice actual."""
        return self.frames[self.current_frame_idx]

    # ------------------------------------------------------------------
    # Generacion del mapa
    # ------------------------------------------------------------------

    def update_map(self) -> None:
        """Genera el mapa Folium para el frame actual y lo guarda en disco."""
        frame = self._current_frame()
        lat = frame.latitude
        lon = frame.longitude
        alt = frame.altitude_rel
        yaw = frame.yaw
        pitch = frame.pitch
        roll = frame.roll

        m = folium.Map(location=[lat, lon], zoom_start=17, tiles='OpenStreetMap')

        # Marcador del dron
        drone_popup = (
            f"<b>DRON</b><br>"
            f"Frame: {self.current_frame_idx + 1}/{len(self.frames)}<br>"
            f"Lat: {lat:.6f}<br>"
            f"Lon: {lon:.6f}<br>"
            f"Alt: {alt:.1f} m<br>"
            f"Yaw: {yaw:.1f} deg<br>"
            f"Pitch: {pitch:.1f} deg<br>"
            f"Roll: {roll:.1f} deg"
        )
        folium.Marker(
            location=[lat, lon],
            popup=folium.Popup(drone_popup, max_width=300),
            icon=folium.Icon(color='red', icon='plane', prefix='fa'),
            tooltip='Dron',
        ).add_to(m)

        # FOV proyectado
        fov_corners = self._calculate_fov_corners(lat, lon, alt, yaw, pitch, roll)

        if len(fov_corners) == 4:
            folium.Polygon(
                locations=fov_corners,
                color='cyan',
                fill=True,
                fill_color='cyan',
                fill_opacity=0.2,
                weight=3,
                popup=f"FOV  Pitch: {pitch:.1f} deg  Yaw: {yaw:.1f} deg",
            ).add_to(m)

            corner_labels = [
                'Top-Left (0, 0)',
                'Top-Right (640, 0)',
                'Bottom-Right (640, 512)',
                'Bottom-Left (0, 512)',
            ]
            for label, (clat, clon) in zip(corner_labels, fov_corners):
                folium.CircleMarker(
                    location=[clat, clon],
                    radius=6,
                    popup=label,
                    color='orange',
                    fill=True,
                    fill_color='orange',
                    fill_opacity=0.8,
                    weight=2,
                    tooltip=label,
                ).add_to(m)

                folium.PolyLine(
                    locations=[[lat, lon], [clat, clon]],
                    color='orange',
                    weight=1,
                    opacity=0.5,
                    dash_array='5, 5',
                ).add_to(m)

        # Circulo de referencia de 500 m
        folium.Circle(
            location=[lat, lon],
            radius=500,
            color='green',
            fill=False,
            weight=1,
            opacity=0.3,
            popup='Rango 500 m',
        ).add_to(m)

        # Panel de informacion
        info_html = (
            '<div style="'
            'font-family: monospace; background: white; padding: 10px; '
            'border-radius: 5px; font-size: 12px;'
            '">'
            f'<b>Frame:</b> {self.current_frame_idx + 1}/{len(self.frames)}<br>'
            f'<b>Lat:</b> {lat:.6f}<br>'
            f'<b>Lon:</b> {lon:.6f}<br>'
            f'<b>Alt:</b> {alt:.1f} m<br>'
            f'<b>Yaw:</b> {yaw:.1f} deg<br>'
            f'<b>Pitch:</b> {pitch:.1f} deg<br>'
            f'<b>Roll:</b> {roll:.1f} deg<br>'
            f'<b>FOV H:</b> {self.geo_calc.fov_h:.1f} deg<br>'
            f'<b>FOV V:</b> {self.geo_calc.fov_v:.1f} deg<br>'
            '</div>'
        )
        m.get_root().html.add_child(folium.Element(info_html))

        map_file = self.OUTPUT_DIR / 'fov_map.html'
        m.save(str(map_file))
        logger.info("Mapa guardado: %s", map_file)

    def _calculate_fov_corners(
        self,
        lat: float,
        lon: float,
        alt: float,
        yaw: float,
        pitch: float,
        roll: float,
    ) -> list:
        """
        Calcula las coordenadas GPS de las 4 esquinas del FOV de la camara.

        Las esquinas se definen en el espacio de imagen usando las dimensiones
        de CAMERA_SPECS para no duplicar constantes.

        Args:
            lat, lon: Posicion del dron en grados decimales.
            alt: Altitud relativa del dron en metros.
            yaw, pitch, roll: Angulos del gimbal en grados.

        Returns:
            Lista de [lat, lon] para cada esquina (top-left, top-right,
            bottom-right, bottom-left). Puede tener menos de 4 elementos si
            alguna conversion falla.
        """
        w = GeoCalculator.CAMERA_SPECS['frame_width']
        h = GeoCalculator.CAMERA_SPECS['frame_height']

        corners_px = [
            (0, 0),     # Top-left
            (w, 0),     # Top-right
            (w, h),     # Bottom-right
            (0, h),     # Bottom-left
        ]

        corners_gps = []
        for px, py in corners_px:
            location = self.geo_calc.pixel_to_gps(
                bbox_center_x=px,
                bbox_center_y=py,
                latitude_drone=lat,
                longitude_drone=lon,
                altitude_drone=alt,
                yaw=yaw,
                pitch=pitch,
                roll=roll,
            )
            if location is not None:
                corners_gps.append([location.latitude, location.longitude])
                logger.debug(
                    "  Corner (%d, %d): (%.6f, %.6f)  %.1f m",
                    px, py, location.latitude, location.longitude, location.distance_m,
                )
            else:
                logger.warning("Calculo GPS fallido para corner (%d, %d).", px, py)
                corners_gps.append([lat, lon])

        return corners_gps

    # ------------------------------------------------------------------
    # Servidor HTTP
    # ------------------------------------------------------------------

    def _start_server(self) -> None:
        """Inicia un servidor HTTP en background para servir los mapas HTML."""
        output_dir = str(self.OUTPUT_DIR)

        class _Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=output_dir, **kwargs)

            def log_message(self, format, *args):
                pass  # Silenciar logs del servidor HTTP

        server = HTTPServer(('localhost', 8000), _Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        logger.info("Servidor HTTP en http://localhost:8000")

    # ------------------------------------------------------------------
    # Interfaz interactiva
    # ------------------------------------------------------------------

    def interactive_loop(self) -> None:
        """Bucle interactivo de navegacion por frames en la terminal."""
        webbrowser.open('http://localhost:8000/fov_map.html')

        print(
            "\n"
            "FOV MAP VISUALIZER\n"
            "------------------\n"
            "Abre http://localhost:8000/fov_map.html en el navegador.\n"
            "\n"
            "Controles:\n"
            "  n  Siguiente frame\n"
            "  p  Frame anterior\n"
            "  j  Ir a frame especifico\n"
            "  h  Mostrar ayuda\n"
            "  q  Salir\n"
        )

        while True:
            key = input('> ').strip().lower()

            if key == 'q':
                print("Saliendo.")
                break
            elif key == 'n':
                self.current_frame_idx = min(
                    self.current_frame_idx + 1, len(self.frames) - 1
                )
                print(f"Frame {self.current_frame_idx + 1}/{len(self.frames)}")
                self.update_map()
            elif key == 'p':
                self.current_frame_idx = max(self.current_frame_idx - 1, 0)
                print(f"Frame {self.current_frame_idx + 1}/{len(self.frames)}")
                self.update_map()
            elif key == 'j':
                raw = input(f"Ir a frame (1-{len(self.frames)}): ")
                try:
                    idx = int(raw) - 1
                    self.current_frame_idx = max(0, min(idx, len(self.frames) - 1))
                    print(f"Frame {self.current_frame_idx + 1}/{len(self.frames)}")
                    self.update_map()
                except ValueError:
                    print("Input invalido. Ingresa un numero entero.")
            elif key == 'h':
                self._print_help()
            else:
                print("Comando no reconocido. Escribe 'h' para ayuda.")

    def _print_help(self) -> None:
        """Muestra la ayuda detallada del visualizador."""
        print(
            "\n"
            "AYUDA\n"
            "-----\n"
            "Comandos disponibles:\n"
            "  n  Avanzar al siguiente frame\n"
            "  p  Retroceder al frame anterior\n"
            "  j  Saltar a un frame especifico (solicita numero)\n"
            "  h  Mostrar esta ayuda\n"
            "  q  Salir del programa\n"
            "\n"
            "Interpretacion del mapa:\n"
            "  Punto rojo       Posicion del dron\n"
            "  Poligono cyan    Area visible por la camara (FOV proyectado)\n"
            "  Puntos naranja   4 esquinas del FOV:\n"
            "                     Top-Left (0, 0)\n"
            "                     Top-Right (640, 0)\n"
            "                     Bottom-Right (640, 512)\n"
            "                     Bottom-Left (0, 512)\n"
            "  Lineas punteadas Conexion del dron a cada esquina (referencia)\n"
            "\n"
            "Que buscar:\n"
            "  OK   FOV es un rectangulo logico\n"
            "  OK   Se rota correctamente con el yaw\n"
            "  OK   Se agranda al aumentar el pitch o bajar la altitud\n"
            "  MAL  Poligono roto o esquinas en posiciones absurdas\n"
            "  MAL  Esquinas a mas de 1 km del dron a altitud tipica\n"
            "  MAL  FOV apunta en la direccion opuesta al yaw\n"
        )


def main() -> None:
    """Punto de entrada principal."""
    if len(sys.argv) > 1:
        srt_path = Path(sys.argv[1])
    else:
        print("Uso: python3 fov_visualizer.py /ruta/al/archivo.SRT")
        srt_path = Path(input("Ingresa la ruta del archivo SRT: ").strip())

    if not srt_path.exists():
        print(f"Archivo SRT no encontrado: {srt_path}")
        raw = input("Ingresa la ruta del archivo SRT: ").strip()
        srt_path = Path(raw)

    if not srt_path.exists():
        logger.error("Archivo no encontrado: %s", srt_path)
        sys.exit(1)

    try:
        visualizer = FOVMapVisualizer(str(srt_path))
        visualizer.interactive_loop()
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")


if __name__ == '__main__':
    main()

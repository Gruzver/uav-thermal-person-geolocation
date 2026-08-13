"""
drone_tracker.launch.py

Lanza el pipeline completo de deteccion y mapeo de personas con camara
termica en dron:

    video_publisher  ->  yolo_detection  ->  georeferencing  ->  map_server

Uso basico (video por defecto):
    ros2 launch drone_bringup drone_tracker.launch.py

Seleccionar video:
    ros2 launch drone_bringup drone_tracker.launch.py \\
        video_path:=/ruta/al/video.MP4

El archivo SRT se deduce automaticamente del video_path reemplazando la
extension por .SRT. Si el nombre del SRT difiere, indicarlo explicitamente:
    ros2 launch drone_bringup drone_tracker.launch.py \\
        video_path:=/ruta/al/video.MP4 \\
        srt_path:=/ruta/al/telemetria.SRT

Argumentos disponibles:
    video_path          Ruta al archivo MP4 termico.
    srt_path            Ruta al SRT (por defecto: mismo path que video con .SRT).
    publish_rate        Hz de publicacion de video (default: 30).
    conf_threshold      Umbral de confianza YOLO (default: 0.5).
    iou_threshold       Umbral IoU para NMS (default: 0.45).
    use_half            FP16 en YOLO: true/false (default: false).
    max_frame_age_ms    Edad maxima de un frame antes de descartarlo, en ms (default: 3600000.0, sin limite).
    image_qos_depth     Profundidad del buffer QoS de imagen en yolo_detector (default: 2000).
    fov_horizontal      FOV horizontal de la camara en grados (default: 50.5).
    max_distance        Distancia maxima GPS aceptable en metros (default: 1000.0).
    min_distance        Distancia minima GPS aceptable en metros (default: 10.0).
    max_sync_delta_ms   Desfase maximo entre deteccion y estado del dron en ms (default: 500.0).
    use_kalman          Activar filtro de Kalman para suavizar GPS: true/false (default: true).
    keep_history        Guardar historial de trayectorias: true/false (default: false).
    output_dir          Directorio de salida para JSON e imagen (default: /tmp/drone_map_data).
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Sin video por defecto: los datos de vuelo no forman parte de este repositorio.
# Indicar video_path:=/ruta/al/clip.MP4 al lanzar.
_DEFAULT_VIDEO = ''


def _derive_srt_path(context, *args, **kwargs):
    """
    Funcion opaca que resuelve los paths de video y SRT en tiempo de launch.

    Si srt_path no fue indicado por el usuario (queda como cadena vacia),
    se deriva automaticamente del video_path reemplazando la extension por
    .SRT (comportamiento estandar de los archivos DJI).

    Devuelve la lista de acciones a ejecutar tras resolver los paths.
    """
    video_path = LaunchConfiguration('video_path').perform(context)
    srt_path = LaunchConfiguration('srt_path').perform(context)

    if not srt_path:
        srt_path = str(Path(video_path).with_suffix('.SRT'))

    log_video = LogInfo(msg=f'[drone_tracker] Video:  {video_path}')
    log_srt = LogInfo(msg=f'[drone_tracker] SRT:    {srt_path}')
    log_sep = LogInfo(msg='[drone_tracker] Iniciando pipeline...')

    conf_threshold = LaunchConfiguration('conf_threshold').perform(context)
    iou_threshold = LaunchConfiguration('iou_threshold').perform(context)
    use_half = LaunchConfiguration('use_half').perform(context)
    max_frame_age_ms = LaunchConfiguration('max_frame_age_ms').perform(context)
    image_qos_depth = LaunchConfiguration('image_qos_depth').perform(context)
    publish_rate = LaunchConfiguration('publish_rate').perform(context)
    fov_horizontal = LaunchConfiguration('fov_horizontal').perform(context)
    max_distance = LaunchConfiguration('max_distance').perform(context)
    min_distance = LaunchConfiguration('min_distance').perform(context)
    max_sync_delta_ms = LaunchConfiguration('max_sync_delta_ms').perform(context)
    use_kalman = LaunchConfiguration('use_kalman').perform(context)
    keep_history = LaunchConfiguration('keep_history').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)

    node_video_publisher = Node(
        package='video_publisher_node',
        executable='video_publisher',
        name='video_publisher',
        output='screen',
        parameters=[{
            'video_path':      video_path,
            'srt_path':        srt_path,
            'publish_rate':    int(publish_rate),
            'use_compression': True,
        }],
    )

    node_yolo_detector = Node(
        package='yolo_detection_node',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'conf_threshold':   float(conf_threshold),
            'iou_threshold':    float(iou_threshold),
            'use_half':         use_half.lower() == 'true',
            'max_frame_age_ms': float(max_frame_age_ms),
            'image_qos_depth':  int(image_qos_depth),
        }],
    )

    node_georeferencer = Node(
        package='georeferencing_node',
        executable='georeferencer',
        name='georeferencer',
        output='screen',
        parameters=[{
            'fov_horizontal':      float(fov_horizontal),
            'max_person_distance': float(max_distance),
            'min_person_distance': float(min_distance),
            'max_sync_delta_ms':   float(max_sync_delta_ms),
            'use_kalman':          use_kalman.lower() == 'true',
        }],
    )

    node_map_server = Node(
        package='map_server_node',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'output_dir':   output_dir,
            'keep_history': keep_history.lower() == 'true',
            'update_rate':  30,
        }],
    )

    return [
        log_video,
        log_srt,
        log_sep,
        node_video_publisher,
        node_yolo_detector,
        node_georeferencer,
        node_map_server,
    ]


def generate_launch_description():
    """Genera el LaunchDescription con todos los argumentos y nodos del pipeline."""

    return LaunchDescription([

        # --- Fuente de datos ---
        DeclareLaunchArgument(
            'video_path',
            default_value=_DEFAULT_VIDEO,
            description='Ruta al archivo de video termico MP4.',
        ),
        DeclareLaunchArgument(
            'srt_path',
            default_value='',
            description=(
                'Ruta al archivo SRT de telemetria DJI. '
                'Si se deja vacio, se deduce del video_path (.MP4 -> .SRT).'
            ),
        ),

        # --- Video publisher ---
        DeclareLaunchArgument(
            'publish_rate',
            default_value='30',
            description='Frecuencia de publicacion de frames en Hz.',
        ),

        # --- YOLO detector ---
        DeclareLaunchArgument(
            'conf_threshold',
            default_value='0.5',
            description='Umbral de confianza minimo para aceptar una deteccion (0.0 - 1.0).',
        ),
        DeclareLaunchArgument(
            'iou_threshold',
            default_value='0.45',
            description='Umbral IoU para Non-Maximum Suppression (0.0 - 1.0).',
        ),
        DeclareLaunchArgument(
            'use_half',
            default_value='false',
            description='Habilitar inferencia FP16 en GPU (true/false).',
        ),
        DeclareLaunchArgument(
            'max_frame_age_ms',
            default_value='3600000.0',
            description=(
                'Edad maxima de un frame en milisegundos antes de descartarlo. '
                'En modo vivo conviene un valor bajo (ej. 100 ms) para evitar lag '
                'acumulado. Este launch es para batch offline: el default es '
                'efectivamente "sin limite" porque aqui queremos procesar cada '
                'frame del clip, no simular tiempo real.'
            ),
        ),
        DeclareLaunchArgument(
            'image_qos_depth',
            default_value='2000',
            description=(
                'Profundidad del QoS de la suscripcion de imagen en yolo_detector. '
                'En modo vivo, 1 es correcto (siempre el frame mas reciente). Aqui, '
                'para batch offline, el default es alto para no perder frames '
                'intermedios por overflow del buffer cuando la inferencia es mas '
                'lenta que el publish_rate -- eso rompe la continuidad del tracker.'
            ),
        ),

        # --- Georeferencing ---
        DeclareLaunchArgument(
            'fov_horizontal',
            default_value='50.5',
            description='FOV horizontal de la camara termica en grados.',
        ),
        DeclareLaunchArgument(
            'max_distance',
            default_value='1000.0',
            description='Distancia horizontal maxima aceptable desde el dron en metros.',
        ),
        DeclareLaunchArgument(
            'min_distance',
            default_value='10.0',
            description='Distancia horizontal minima aceptable desde el dron en metros.',
        ),
        DeclareLaunchArgument(
            'max_sync_delta_ms',
            default_value='500.0',
            description=(
                'Desfase maximo aceptable en milisegundos entre una deteccion y el '
                'estado del dron mas cercano en el tiempo. Si se supera, el frame '
                'se descarta.'
            ),
        ),
        DeclareLaunchArgument(
            'use_kalman',
            default_value='true',
            description='Activar filtro de Kalman para suavizar coordenadas GPS (true/false).',
        ),

        # --- Map server ---
        DeclareLaunchArgument(
            'keep_history',
            default_value='false',
            description='Guardar historial de posiciones por persona (true/false).',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/tmp/drone_map_data',
            description='Directorio donde se escriben map_data.json y current_frame.jpg.',
        ),

        # Resolucion de paths y lanzamiento de nodos
        OpaqueFunction(function=_derive_srt_path),
    ])

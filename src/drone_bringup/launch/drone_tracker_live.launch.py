"""
drone_tracker_live.launch.py

Lanza el pipeline de deteccion y mapeo en TIEMPO REAL con camara termica DJI
Mavic 3T y telemetria via MQTT.

Arquitectura:
    camera_publisher ─┐
                      ├─> yolo_detection  ->  georeferencing  ->  map_server
    mqtt_telemetry  ──┘

Reemplaza drone_tracker.launch.py (pipeline offline con MP4+SRT).
El resto del pipeline (YOLO, georeferenciador, map server) es identico.

Uso basico:
    ros2 launch drone_bringup drone_tracker_live.launch.py

Con capturadora en otro indice de dispositivo:
    ros2 launch drone_bringup drone_tracker_live.launch.py device_index:=2

Con ruta estable del dispositivo:
    ros2 launch drone_bringup drone_tracker_live.launch.py device_path:=/dev/v4l/by-id/usb-UGREEN_25854-video-index0

NOTA sobre sincronizacion:
    El MQTT OSD llega cada ~2 segundos (0.5 Hz). El parametro max_sync_delta_ms
    se ajusta a 1100 ms para que el georeferenciador no descarte frames entre
    actualizaciones de telemetria. En el pipeline offline (SRT a 30 Hz) el
    default de 500 ms es suficiente.

Argumentos disponibles:
    device_path         Ruta V4L2 de la capturadora (default: /dev/video3).
    device_index        Indice si device_path esta vacio (default: 3).
    frame_width         Ancho de captura en pixeles (default: 1920).
    frame_height        Alto de captura en pixeles (default: 1080).
    fps                 FPS solicitados al dispositivo (default: 30).
    pixel_format        FourCC solicitado al dispositivo (default: MJPG).
    publish_rate        Hz de publicacion de video (default: 30).
    broker_host         Host del broker MQTT EMQX (default: 127.0.0.1).
    broker_port         Puerto del broker MQTT (default: 1883).
    aircraft_sn         Numero de serie del Mavic 3T.
    focal_len           Focal length de la camara termica en mm (default: 40.0).
    conf_threshold      Umbral de confianza YOLO (default: 0.5).
    iou_threshold       Umbral IoU para NMS (default: 0.45).
    use_half            FP16 en YOLO: true/false (default: false).
    max_frame_age_ms    Edad maxima de frame antes de descartar (default: 100.0).
    fov_horizontal      FOV horizontal de la camara en grados (default: 50.5).
    max_distance        Distancia maxima GPS aceptable en metros (default: 1000.0).
    min_distance        Distancia minima GPS aceptable en metros (default: 10.0).
    use_kalman          Activar filtro de Kalman para suavizar GPS: true/false (default: true).
    keep_history        Guardar historial de trayectorias: true/false (default: false).
    output_dir          Directorio de salida para JSON e imagen (default: /tmp/drone_map_data).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_DEFAULT_AIRCRAFT_SN = ''


def _launch_nodes(context, *args, **kwargs):
    """Resuelve parametros y devuelve los nodos del pipeline live."""

    device_path = LaunchConfiguration('device_path').perform(context)
    device_index = LaunchConfiguration('device_index').perform(context)
    frame_width = LaunchConfiguration('frame_width').perform(context)
    frame_height = LaunchConfiguration('frame_height').perform(context)
    fps = LaunchConfiguration('fps').perform(context)
    pixel_format = LaunchConfiguration('pixel_format').perform(context)
    publish_rate = LaunchConfiguration('publish_rate').perform(context)
    broker_host = LaunchConfiguration('broker_host').perform(context)
    broker_port = LaunchConfiguration('broker_port').perform(context)
    aircraft_sn = LaunchConfiguration('aircraft_sn').perform(context)
    focal_len = LaunchConfiguration('focal_len').perform(context)
    conf_threshold = LaunchConfiguration('conf_threshold').perform(context)
    iou_threshold = LaunchConfiguration('iou_threshold').perform(context)
    use_half = LaunchConfiguration('use_half').perform(context)
    max_frame_age_ms = LaunchConfiguration('max_frame_age_ms').perform(context)
    fov_horizontal = LaunchConfiguration('fov_horizontal').perform(context)
    max_distance = LaunchConfiguration('max_distance').perform(context)
    min_distance = LaunchConfiguration('min_distance').perform(context)
    use_kalman = LaunchConfiguration('use_kalman').perform(context)
    keep_history = LaunchConfiguration('keep_history').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)

    node_camera_publisher = Node(
        package='camera_publisher_node',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'device_path':     device_path,
            'device_index':    int(device_index),
            'frame_width':     int(frame_width),
            'frame_height':    int(frame_height),
            'fps':             int(float(fps)),
            'publish_rate':    int(publish_rate),
            'pixel_format':    pixel_format,
            'use_compression': False,
        }],
    )

    node_mqtt_telemetry = Node(
        package='mqtt_telemetry_node',
        executable='mqtt_telemetry',
        name='mqtt_telemetry',
        output='screen',
        parameters=[{
            'broker_host': broker_host,
            'broker_port': int(broker_port),
            # Credenciales vacias a proposito: el nodo las toma de las variables
            # de entorno DJI_MQTT_USERNAME / DJI_MQTT_PASSWORD.
            'username':    '',
            'password':    '',
            'aircraft_sn': aircraft_sn,
            'focal_len':   float(focal_len),
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
            'use_kalman':          use_kalman.lower() == 'true',
            # MQTT OSD llega cada ~2s -> maximo desfase entre estados es ~1000ms.
            # Se usa 1100ms para tener margen ante variaciones de red.
            'max_sync_delta_ms':   1100.0,
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
        LogInfo(msg='[drone_tracker_live] Iniciando pipeline en tiempo real...'),
        LogInfo(msg=f'[drone_tracker_live] Capturadora: {device_path or "/dev/video" + device_index}'),
        LogInfo(msg=f'[drone_tracker_live] Broker MQTT: {broker_host}:{broker_port}'),
        node_camera_publisher,
        node_mqtt_telemetry,
        node_yolo_detector,
        node_georeferencer,
        node_map_server,
    ]


def generate_launch_description():
    return LaunchDescription([

        # --- Camara en tiempo real ---
        DeclareLaunchArgument(
            'device_path',
            default_value='/dev/video3',
            description='Ruta V4L2 de la capturadora USB. Si esta vacia, se usa device_index.',
        ),
        DeclareLaunchArgument(
            'device_index',
            default_value='3',
            description='Indice de la capturadora USB si device_path esta vacio.',
        ),
        DeclareLaunchArgument(
            'frame_width',
            default_value='1920',
            description='Ancho de captura solicitado al dispositivo en pixeles.',
        ),
        DeclareLaunchArgument(
            'frame_height',
            default_value='1080',
            description='Alto de captura solicitado al dispositivo en pixeles.',
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30',
            description='FPS solicitados al dispositivo.',
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='MJPG',
            description='FourCC solicitado al dispositivo.',
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='30',
            description='Frecuencia de publicacion de frames en Hz.',
        ),

        # --- Telemetria MQTT ---
        DeclareLaunchArgument(
            'broker_host',
            default_value='127.0.0.1',
            description='Host del broker MQTT EMQX.',
        ),
        DeclareLaunchArgument(
            'broker_port',
            default_value='1883',
            description='Puerto del broker MQTT EMQX.',
        ),
        DeclareLaunchArgument(
            'aircraft_sn',
            default_value=_DEFAULT_AIRCRAFT_SN,
            description='Numero de serie del Mavic 3T (define el topico MQTT).',
        ),
        DeclareLaunchArgument(
            'focal_len',
            default_value='40.0',
            description='Focal length de la camara termica en mm (Mavic 3T: 40.0).',
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
            default_value='100.0',
            description='Edad maxima de un frame en ms antes de descartarlo.',
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

        OpaqueFunction(function=_launch_nodes),
    ])

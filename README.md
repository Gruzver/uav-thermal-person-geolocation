# Thermal Person Geolocation for UAV-Based Search and Rescue

A modular ROS2 pipeline for real-time thermal person detection and GPS geolocation from a
gimbal-stabilized UAV camera. Offline (recorded video plus telemetry) and live (DJI Cloud API)
data sources populate an identical message-topic interface, so no detection or geolocation code
changes between the two modes.

Companion code for the paper *"A Modular ROS2 Pipeline for Sub-Meter Thermal Person Geolocation in
UAV-Based Search and Rescue"* — Pontificia Universidad Católica del Perú (PUCP), currently under
review. Platform: DJI Mavic 3T, ROS2 Humble.

---

## Results

Detection, on a held-out test set of 737 thermal images:

| Metric | Value |
|---|---|
| mAP@0.5 | 0.858 |
| mAP@0.5:0.95 | 0.474 |
| Precision | 0.927 |
| Recall | 0.877 |

The detector is a YOLOv8x model fine-tuned via two-stage transfer learning on 4,910 annotated
thermal frames drawn from 20 real flight sessions (single `person` class; 3,437 / 736 / 737
train / val / test split).

Geolocation, validated against an independently surveyed stationary reference target across a
controlled sweep of 32 flight passes — four altitudes (15, 25, 35, 50 m) × two gimbal angles
(nadir −90°, oblique −45°) × four ground-speed levels (2.0–4.8 m/s):

| Gimbal | Mean error | Passes | Range |
|---|---|---|---|
| Oblique (−45°) | 0.64 m | 16 / 16 | 0.25 – 0.97 m |
| Near-nadir (−90°) | 0.64 m | 11 / 15 | 0.17 – 1.10 m |

**On the near-nadir figure.** Passes yielding fewer than 10 gate-passing detections are excluded
from the aggregate, on the grounds that a mean computed over a handful of frames is not a
meaningful estimate. Four of the fifteen near-nadir passes fall below that threshold — three at
50 m and one at 35 m, where target range approaches the detector's effective limit — and they are
excluded. Those four passes carry large errors, so the unfiltered near-nadir mean over all fifteen
passes is 3.92 m rather than 0.64 m. Both numbers are stated here deliberately: the 0.64 m figure
describes accuracy where detection is reliable, not accuracy unconditional on detection. The
oblique configuration needs no such filter — all sixteen passes qualify.

The two viewing geometries are otherwise statistically indistinguishable, which the pipeline's
sensitivity model explains: for a fixed angular calibration error `δθ`, the ground-distance error
propagates as

```
|∂d/∂θ| = h / sin²θ
```

so at equal altitude the oblique boresight (θ = −45°) is exactly twice as sensitive as near-nadir
viewing (θ = −90°). Near-nadir is, by this model, the more robust geometry rather than the more
fragile one.

---

## Architecture

Nine ROS2 packages. Custom message types (`DroneState`, `PersonDetection[Array]`,
`PersonLocation[Array]`) define the interface between stages, which is what lets the offline and
live source nodes be swapped without touching anything downstream.

```
 offline:  video_publisher ─┐
                            ├─> yolo_detection ─> georeferencing ─> map_server
 live:     camera_publisher ┤
           mqtt_telemetry  ─┘
```

| Package | Role |
|---|---|
| [`drone_bringup`](src/drone_bringup) | Launch files; selects offline / live / NRT configuration |
| [`drone_tracker_msgs`](src/drone_tracker_msgs) | Custom message definitions |
| [`drone_tracker_utils`](src/drone_tracker_utils) | `GeoCalculator` (pixel→GPS), SRT telemetry parser, `KalmanTracker`, FOV visualizer |
| [`video_publisher_node`](src/video_publisher_node) | Offline source: MP4 video + SRT telemetry |
| [`camera_publisher_node`](src/camera_publisher_node) | Live source: USB capture card or RTSP stream |
| [`mqtt_telemetry_node`](src/mqtt_telemetry_node) | Live telemetry via DJI Cloud API (MQTT), replaces SRT |
| [`yolo_detection_node`](src/yolo_detection_node) | YOLOv8 inference and multi-object tracking |
| [`georeferencing_node`](src/georeferencing_node) | Pixel-to-GPS projection, Kalman filtering |
| [`map_server_node`](src/map_server_node) | Aggregation and export of georeferenced positions |

The pixel-to-GPS projection is implemented in
[`geo_calculator.py`](src/drone_tracker_utils/drone_tracker_utils/geo_calculator.py). It composes
the pixel's angular offset within the camera FOV with the gimbal attitude, intersects the
resulting ray with the ground plane, and converts the offset to a latitude/longitude delta. The
formulation stays valid across the full angular range, including rays that rotate past pure
nadir, where the azimuth is rotated by 180° to follow the ray's true heading.

### Live-mode telemetry constraint

The DJI Cloud API delivers onboard-state telemetry, gimbal orientation included, at a fixed
0.5 Hz that is not user-configurable. Since georeferencing needs gimbal yaw/pitch/roll at the
detection timestamp, live-mode error grows with elapsed time since the last update and with drone
velocity, approximately as `e ≈ v · Δt` with `Δt ≤ 2 s`. This is why `georeferencer_nrt` exists:
it withholds geolocation until the drone's position and orientation have been stable for a
configurable window, then uses the most recent telemetry sample without incurring that error.

---

## Installation

Requires **ROS2 Humble** on Ubuntu 22.04.

```bash
# Dependencies
sudo apt install ros-humble-desktop ros-humble-cv-bridge ros-humble-usb-cam
pip install ultralytics opencv-python numpy paho-mqtt folium

# Build
git clone https://github.com/Gruzver/uav-thermal-person-geolocation.git
cd uav-thermal-person-geolocation
colcon build --symlink-install
source install/setup.bash
```

### Trained weights

The fine-tuned YOLOv8x weights are **not** in this repository — the file exceeds GitHub's 100 MB
per-file limit. Download them from the [Releases](../../releases) page and pass the path via the
`model_path` parameter. The node fails immediately with an explicit message if that path is unset,
rather than starting and silently producing nothing.

Flight data (thermal video, SRT telemetry, annotated dataset) is likewise not included.

---

## Usage

**Offline** — recorded video with its SRT telemetry. This is the mode that reproduces the paper's
results:

```bash
ros2 launch drone_bringup drone_tracker.launch.py \
    video_path:=/path/to/clip.MP4 \
    model_path:=/path/to/best.pt
```

The SRT path is derived from the video path automatically; override it with `srt_path:=` when the
two differ.

**Live** — USB capture card for video, DJI Cloud API for telemetry:

```bash
export DJI_MQTT_USERNAME=<user>
export DJI_MQTT_PASSWORD=<password>
export DJI_AIRCRAFT_SN=<serial>

ros2 launch drone_bringup drone_tracker_live.launch.py \
    broker_host:=<mqtt-broker> \
    model_path:=/path/to/best.pt
```

**NRT** — RTSP video with stability-gated georeferencing, for the 0.5 Hz telemetry case described
above:

```bash
ros2 launch drone_bringup drone_tracker_nrt.launch.py \
    broker_host:=<mqtt-broker> \
    mediamtx_api:=http://<mediamtx-host>:9997 \
    model_path:=/path/to/best.pt \
    stability_window_s:=4.0
```

Credentials and the aircraft serial have no defaults and are never stored in this repository. Pass
them as ROS2 parameters or through the environment variables shown above.

### Key parameters

| Parameter | Node | Default | Meaning |
|---|---|---|---|
| `fov_horizontal` | `georeferencer` | 50.5 | Camera horizontal field of view, degrees |
| `max_sync_delta_ms` | `georeferencer` | 500 | Max detection-to-telemetry offset before dropping a frame |
| `use_kalman` | `georeferencer` | true | Constant-velocity smoothing of per-track GPS |
| `conf_threshold` | `yolo_detector` | 0.5 | Detection confidence threshold |
| `stability_window_s` | `georeferencer_nrt` | 4.0 | Seconds of stable attitude required before geolocating |

---

## Citation

The paper is under review; this section will be updated with the full reference once proceedings
are assigned. In the meantime, cite this repository directly.

## License

MIT — see [LICENSE](LICENSE).

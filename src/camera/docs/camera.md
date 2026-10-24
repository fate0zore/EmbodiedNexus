# Camera adapter (ROS 2 Humble)

The first camera adapter targets the Gemini335 on the Yahboom robot. It provides a standalone ROS 2 node, a small authenticated HTTP/WebSocket gateway, a ROS-independent Python client, point-cloud generation and an RGB AprilTag calibration workflow.

## Build and start on the robot

Run repository-relative commands from the EmbodiedNexus root.

```bash
cd ~/EmbodiedNexus
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to embodied_nexus_camera
source install/setup.bash
export EMBODIED_NEXUS_CAMERA_TOKEN='replace-with-a-local-token'
ros2 launch embodied_nexus_camera camera.launch.py \
  config_file:=$(pwd)/src/camera/embodied_nexus_camera/config/gemini335.yaml
```

Use `interface_mode:=ros2`, `web`, or `both`. The Web API listens on the configured port (default 8765). The node always uses ROS 2 internally. It never starts or stops another camera process; the V4L2 device must be free before selecting the `gemini_v4l2` backend.

To consume existing Arachne ROS 2 image topics without opening V4L2 a second time, set `source.backend: ros2_topics` and map `source.*_topic` to the active camera topics. Configure a distinct `pointcloud.input_topic` when `pointcloud.source: ros2_topic` to avoid a feedback loop.

## ROS 2 outputs

Image, CameraInfo and PointCloud2 publishers use the `sensor_data` best-effort QoS profile. Subscribers need compatible best-effort QoS.

```text
/embodied_nexus/cameras/camera_front/color/image_raw
/embodied_nexus/cameras/camera_front/color/camera_info
/embodied_nexus/cameras/camera_front/depth/image_raw
/embodied_nexus/cameras/camera_front/depth/camera_info
/embodied_nexus/cameras/camera_front/points
/embodied_nexus/cameras/camera_front/status
```

Configuration services are `config/get`, `config/validate`, and `config/apply`; request and response payloads use JSON strings. Only documented runtime-safe fields can be changed online; capture and source changes require restart. Calibration services are `calibration/start`, `calibration/capture`, and `calibration/solve`; the `calibration/solve_action` action reports progress and accepts cancellation. Publish a candidate ID to `calibration/apply` to explicitly apply it. The result and current state are published on `calibration/status`. Runtime-safe parameters are namespaced, for example `streams.jpeg_quality` and `pointcloud.stride`.

## Web API

All HTTP requests require `Authorization: Bearer <token>`. The WebSocket upgrade uses the subprotocols `en-camera-v1` and `bearer.<token>`; the token is not placed in the URL.

```text
GET  /api/v1/cameras/camera_front/info
GET  /api/v1/cameras/camera_front/status
GET  /api/v1/cameras/camera_front/config
PUT  /api/v1/cameras/camera_front/config
GET  /api/v1/cameras/camera_front/snapshot.jpg
GET  /api/v1/cameras/camera_front/calibration
POST /api/v1/cameras/camera_front/calibration/sessions
POST /api/v1/cameras/camera_front/calibration/sessions/{id}/capture
DELETE /api/v1/cameras/camera_front/calibration/sessions/{id}/samples/{frame_name}
POST /api/v1/cameras/camera_front/calibration/sessions/{id}/solve
GET  /api/v1/cameras/camera_front/calibration/results/{id}
POST /api/v1/cameras/camera_front/calibration/results/{id}/apply
GET  /api/v1/cameras/camera_front/calibration/history
POST /api/v1/cameras/camera_front/calibration/history/{id}/restore
WS   /ws?streams=color,depth,points&color_fps=15&depth_fps=5&color_codec=jpeg
```

Each WebSocket frame consists of a UTF-8 JSON metadata message followed by one binary message. Color is JPEG by default (PNG is supported), depth uses lossless PNG16, and points use the `PointCloud2` binary layout described by the metadata. Per-client latest-frame delivery is bounded and skips stale queued frames.

Every frame header includes its `channel`, `stamp_sec`, `timestamp_source`, `calibration_version`, and `frame_id`; depth frames also include `depth_scale_m`. Pair color and depth only when their timestamp sources and calibration versions match and their timestamps fall within the consumer's configured tolerance. The calibration endpoint reports both streams' image size, intrinsics, distortion coefficients/model and calibrated flag, plus the depth scale and depth-to-color rotation/translation in meters.

## Python client

From the repository root on the Windows/Linux algorithm host, run `pip install -e sdk/python`. The distribution is `embodied-nexus-client`, with imports under `embodied_nexus_client`. The client uses HTTP from the standard library and `websockets` for live frames; it does not require ROS 2.

```python
from embodied_nexus_client import CameraClient

camera = CameraClient("http://192.168.1.102:8765", token="...", camera_id="camera_front")
print(camera.status())
for frame in camera.subscribe(("color", "depth"), color_fps=10, depth_fps=5):
    print(frame.metadata["channel"], frame.metadata["stamp_sec"], len(frame.data))
```

Open `src/camera/examples/camera_websocket.html` from a local HTTP server for a minimal browser preview. Enter the WebSocket URL and token in the page.

## Calibration and point-cloud configuration

The repository includes a sanitized copy of the current Yahboom Gemini335 factory profile. `custom_file`, when set, overrides the factory stream profiles. The original distortion model and all coefficients are retained; unsupported models block XYZ projection while RGB and depth images remain available. Depth-to-color translations are converted from the configured source unit to meters at load time.

Resolution changes do not silently reuse factory intrinsics. `calibration.allow_resolution_scaling` defaults to `false`; set it to `true` only when the new stream is a pure equal-aspect resize. Otherwise the image stream remains available while geometry reports the calibration mismatch. `pointcloud.roi` is an optional depth-image pixel rectangle `[x, y, width, height]`; an empty list uses the full frame, and the ROS configuration service can change it online. `capture.color_controls` and `capture.depth_controls` accept driver-specific numeric V4L2 control maps. Inspect supported names and ranges with `v4l2-ctl --list-ctrls`; capture-control changes require a restart and unsupported controls are reported as capture errors.

RGB calibration uses the existing AprilTag board reference workflow. Set `calibration_session.board_image_path`, measured board width/height, and the output root. Capture samples through the API, solve, inspect the RMS and per-view results, then explicitly apply an accepted candidate. ROS 2 clients can use `/embodied_nexus/cameras/<camera_id>/calibration/solve_action` for progress and cancellation; Web clients can poll the calibration summary and candidate endpoint. Applying writes a versioned active profile without replacing the source factory file. Depth intrinsics, depth scale, and RGB-D extrinsics remain imported/configurable values.

Depth-derived XYZRGB requires valid color and depth models plus a depth-to-color transform. Point generation reports unavailable calibration instead of substituting estimated focal lengths. Select `pointcloud.source: ros2_topic` to preserve an existing point cloud's binary fields and frame metadata. XYZ-only ROS 2 clouds receive an `rgb` field when valid RGB intrinsics, a sufficiently close timestamp and a transform into the RGB optical frame are available; otherwise the original XYZ cloud is forwarded with a colorization reason in its metadata.

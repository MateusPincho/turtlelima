# Camera → segmentation → costmap: transport changes and verification

## What changed and why

When the remote GPU server ran segmentation, `/camera/realsense/color/image_raw` dropped from 15 Hz to about 6 Hz.
The server was subscribing to the **raw** color image over the network: 640x480 rgb8 at 15 Hz is about 14 MB/s (110 Mbit/s).
The camera node's publish path stalled on that link, which slowed every camera topic.
Inference was not the bottleneck.

The `semantic_segmentation_layer` needs, on the robot:

| Input | Type | Notes |
|---|---|---|
| `segmentation_topic` | raw `sensor_msgs/Image` | One class id per pixel. Compressed images are not accepted. |
| `pointcloud_topic` | organized `PointCloud2` | Same width × height as the mask, otherwise the message is dropped. |
| `labels_topic` | `vision_msgs/LabelInfo` | Reliable, transient local. |
| `confidence_topic` | raw `Image` (optional) | If missing, every confidence is set to 255. |

The mask and cloud are paired with ApproximateTime sync (100 ms max interval, queue 10).

New data flow:

```
ROBOT                                                       SERVER (GPU)
realsense ── color/image_raw/compressed (JPEG) ─────────▶ segmentation_image_decoder (compressed→raw)
   │                                                            │ segmentation/image_raw
   │ depth/color/points (organized, stays on robot)             ▼
   │                                                        semantic_segmentation
   ▼                                                            │ mask (raw, local)
semantic_segmentation_layer ◀── /segmentation/mask ◀──          ▼
                     segmentation_mask_decoder ◀── mask/compressed (PNG) ── segmentation_mask_encoder
                         (robot, compressed→raw)
                     label_info (latched, small) ─────────────────────────────────────────▶ robot
```

| File | Change |
|---|---|
| `turtlelima_bringup/launch/camera.launch.py` | Point cloud on and organized. JPEG q80 for `color/image_raw/compressed`. Every other image topic publishes raw only (no theora, zstd or compressedDepth). New `color_qos`, `depth_qos` and `pointcloud_qos` launch args. |
| `turtlelima_segmentation/launch/segmentation.launch.py` | Server side. Decodes the compressed color image locally and publishes `mask/compressed` (PNG, lossless). New args `input_transport` and `compress_mask`. |
| `turtlelima_bringup/launch/segmentation_relay.launch.py` | New, robot side. Decodes `/mask/compressed` into raw `/segmentation/mask` for the layer. |
| `semantic_segmentation_node.py` | The `mask` and `confidence` publishers keep only the newest message (depth 1, was 10) and stay reliable. |
| `docker/Dockerfile`, `docker-compose.yml`, `docker/cyclonedds.xml` | Cyclone DDS is installed and is the default RMW, with a shared config: multicast for discovery only, receive buffer configurable, static peers through `.env`. |

### How to launch

```bash
# Robot
ros2 launch turtlelima_bringup camera.launch.py
ros2 launch turtlelima_bringup segmentation_relay.launch.py
# Nav2 layer config: segmentation_topic: /segmentation/mask
#                    pointcloud_topic:   /camera/realsense/depth/color/points
#                    labels_topic:       /label_info

# Server
ros2 launch turtlelima_segmentation segmentation.launch.py
```

---

## Host setup (once per machine, outside Docker)

Cyclone needs a larger kernel socket receive buffer for image traffic. A container with `network_mode: host` can't change this setting, so set it on the **robot host and the server host**:

```bash
sudo tee /etc/sysctl.d/60-cyclonedds.conf <<'EOF'
net.core.rmem_max=2147483647
net.core.wmem_max=2147483647
net.core.rmem_default=8388608
EOF
sudo sysctl --system
```

Then put `CYCLONEDDS_RCVBUF=10MB` in the `.env` next to `docker-compose.yml`, and recreate the containers (`docker compose up -d --force-recreate`).

If the robot and server are on different subnets or a VPN (no multicast between them), also set the other host's IP in each `.env`:

```bash
ROS_STATIC_PEERS=192.168.1.50          # on the robot: server IP
ROS_STATIC_PEERS=192.168.1.20          # on the server: robot IP
```

If the robot must use specific interfaces (for example Wi-Fi plus the Create 3 link), append an override to `CYCLONEDDS_URI` in the robot's `.env`. Use the interface names from `ip -br addr`:

```bash
CYCLONEDDS_URI=file:///opt/dds/cyclonedds.xml,<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlan0"/><NetworkInterface name="usb0"/></Interfaces></General></Domain></CycloneDDS>
```

---

## Verification on the robot network

Run every command inside the containers (`docker compose exec dev bash`, or `dev-gpu` on the server). Each step says where to run it: **[R]** is the robot, **[S]** is the server.

### 1. DDS configuration [R] [S]

```bash
env | grep -E "RMW|CYCLONE|ROS_"
```
Expected:
```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///opt/dds/cyclonedds.xml        (plus your override, if any)
CYCLONEDDS_RCVBUF=10MB                               (after the host setup)
ROS_DOMAIN_ID=<same value on robot and server>
ROS_STATIC_PEERS=...                                  (empty if same subnet)
```

```bash
ros2 doctor --report | grep -i -A1 middleware
```
Expected: `middleware name : rmw_cyclonedds_cpp`.

```bash
sysctl net.core.rmem_max         # on the host, not in the container
```
Expected: `net.core.rmem_max = 2147483647`.

None of the node logs should contain `failed to increase socket receive buffer size`. If one does, the host limit is lower than `CYCLONEDDS_RCVBUF`.

### 2. Create 3 is still reachable [R] [S]

This step confirms the Cyclone settings did not break the base.

```bash
ros2 topic list | grep -E "odom|battery_state"
ros2 topic hz /odom                 # add the namespace if your Create 3 uses one
ros2 topic echo /battery_state --once
```
Expected: the topics are listed, `/odom` runs at about 20 Hz, and one `BatteryState` message is printed.
If nothing appears, Cyclone chose the wrong interface. Add the `Interfaces` override shown above.
To see which interface Cyclone uses, start any node with `CYCLONEDDS_URI="$CYCLONEDDS_URI,<CycloneDDS><Domain><Tracing><Verbosity>config</Verbosity><OutputFile>stdout</OutputFile></Tracing></Domain></CycloneDDS>"` and look for `interfaces:` in the output.

### 3. Camera parameters took effect [R]

```bash
ros2 param get /camera/realsense pointcloud.enable
ros2 param get /camera/realsense pointcloud.ordered_pc
ros2 param get /camera/realsense pointcloud.pointcloud_qos
ros2 param list /camera/realsense | grep -i qos
ros2 param list /camera/realsense | grep enable_pub_plugins
```
Expected:
```
Boolean value is: True
Boolean value is: True
String value is: SENSOR_DATA
```
- `grep -i qos` shows which naming your wrapper version uses: `color_qos`/`depth_qos` or `rgb_camera.color_qos`/`depth_module.depth_qos`. The launch sets both, and the node ignores the one it doesn't declare.
- `grep enable_pub_plugins` should show entries such as `color.image_raw.enable_pub_plugins`. If nothing is listed, image_transport used another prefix. Step 4 shows whether the plugins were still restricted.

### 4. Topic list: no unused transports [R]

```bash
ros2 topic list | grep /camera/realsense
```
Expected, among others:
```
/camera/realsense/color/image_raw
/camera/realsense/color/image_raw/compressed
/camera/realsense/depth/image_rect_raw
/camera/realsense/aligned_depth_to_color/image_raw
/camera/realsense/depth/color/points
```
```bash
ros2 topic list | grep -E "theora|zstd|compressedDepth"
```
Expected: **no output**. If the topics are still there, `enable_pub_plugins` did not apply. Run `ros2 param list /camera/realsense | grep image_raw` to see the prefix the node really uses, and adjust `_param_bases()` in `camera.launch.py`.

### 5. Point cloud shape and frame [R]

```bash
ros2 topic echo /camera/realsense/depth/color/points --field height --once
ros2 topic echo /camera/realsense/depth/color/points --field width --once
ros2 topic echo /camera/realsense/depth/color/points --field header.frame_id --once
ros2 topic echo /camera/realsense/depth/color/points --field is_dense --once
```
Expected:
```
480
640
realsense_color_optical_frame
false
```
- With an unordered cloud, `height` is `1` and the layer drops every message.
- With `align_depth` on, the frame must be the color optical frame, because the mask is computed on the color image.

### 6. Who subscribes to what [R] [S]

```bash
ros2 topic info -v /camera/realsense/color/image_raw
```
Expected: **no subscriber from the server** (no `segmentation_image_decoder` and no `semantic_segmentation`). The publisher's reliability is `RELIABLE`, or `BEST_EFFORT` if you set `color_qos:=SENSOR_DATA`.

```bash
ros2 topic info -v /camera/realsense/color/image_raw/compressed
```
Expected: subscriber `segmentation_image_decoder`, reliability `RELIABLE`.
> Leave `color_qos` at `SYSTEM_DEFAULT`. With `SENSOR_DATA` the publisher is best effort and the decoder (reliable) can't connect. You would see `Subscription count: 1` on the topic, but the decoder would receive no messages.

If the decoder is listed under the **raw** topic instead, check its log for `The 'in_transport' parameter is set to: compressed`. The republish nodes only subscribe once something reads their output, so the decoder appears here only after `semantic_segmentation` is active.

The republish setup (parameters, remaps, PNG round trip) was tested in a plain `ros:jazzy-ros-base` container with Cyclone. The camera-side `enable_pub_plugins` and JPEG parameters could not be tested without the camera, which is why steps 3, 4 and 8 check them.

```bash
# [S]
ros2 node list
```
Expected: `/segmentation_image_decoder`, `/semantic_segmentation`, `/segmentation_mask_encoder`.

### 7. Rates [R] [S]

Run with the server segmentation **stopped**, then again with it **running**:

```bash
# [R]
ros2 topic hz /camera/realsense/color/image_raw
ros2 topic hz /camera/realsense/depth/color/points
```
Expected: about **15 Hz both times**, which is the target of this change. Before the change, the color topic fell to about 6 Hz with segmentation running.

```bash
# [S]
ros2 topic hz /camera/realsense/color/image_raw/compressed
ros2 topic hz /segmentation/image_raw
ros2 topic hz /mask
```
Expected: about 15 Hz for the first two. `/mask` runs at the inference rate: 15 Hz if inference is under ~66 ms, lower otherwise.

```bash
# [R]
ros2 topic hz /mask/compressed
ros2 topic hz /segmentation/mask
```
Expected: the same rate as `/mask` on the server.

### 8. Bandwidth [R]

```bash
ros2 topic bw /camera/realsense/color/image_raw/compressed
ros2 topic bw /mask/compressed
nload <wifi-interface>     # or: iftop -i <wifi-interface>
```
Expected:
- Compressed color: about **0.5–1 MB/s** (roughly 40–60 KB per message).
- Compressed mask: under 0.5 MB/s.
- Wi-Fi traffic: a few Mbit/s outgoing, instead of the ~110 Mbit/s raw image stream from before.

```bash
ros2 topic echo /mask/compressed --field format --once
```
Expected: a string containing `png compressed`. If it shows `jpeg`, the encoder's format parameter used another prefix. Check it with `ros2 param list /segmentation_mask_encoder | grep format` and fix the name in `segmentation.launch.py`. A JPEG mask corrupts class ids.

### 9. Timestamps and latency [R]

```bash
ros2 topic echo /segmentation/mask --field header --once
```
Expected: `frame_id` is `realsense_color_optical_frame`, the same as the color image. The node copies the image header.
If it shows something else, check that the RealSense publishes the color frame under that name (`ros2 topic echo /camera/realsense/color/image_raw --field header.frame_id --once`).

```bash
ros2 topic delay /segmentation/mask
```
Expected: an average delay well **under 0.6 s** (inference plus two network hops, typically 0.1–0.3 s).
The layer pairs mask and cloud by stamp, with a queue of 10 clouds (about 0.66 s at 15 Hz). A larger delay means the matching cloud has already left the queue and the mask is dropped. If so, raise `sensor_data_depth` in the layer config.
A negative or huge delay means the robot and server clocks are not synchronized. Run `chronyc tracking` on both hosts.

### 10. Costmap layer [R]

This step applies once Nav2 is running with the layer.

```bash
ros2 topic echo /label_info --once        # must arrive (latched)
```
Then check the Nav2 log for these layer warnings:
- `Pointcloud and segmentation sizes are different` means the mask and cloud resolutions differ (step 5).
- `Class map is empty because a labelinfo message has not been received` means `label_info` QoS or the topic name is wrong.

Expected: neither warning appears. With `visualize_tile_map: True`, `ros2 topic hz /<source>/tile_map` follows the mask rate.

---

## Known issue outside this change

The segmentation node publishes `mask` as **rgb8** (a colorized image) and `confidence` as **32FC1** (`semantic_segmentation_node.py`, `process_image`).
The layer expects **mono8** class ids and a **mono8** confidence in 0–255.
An rgb8 mask passes the layer's size check, but the layer reads the wrong bytes as class ids, so the costmap will be wrong until the node publishes mono8.
For the same reason, the confidence image is not relayed to the robot yet. Once it is mono8, add a second decoder to `segmentation_relay.launch.py` and an encoder to `segmentation.launch.py`, as for the mask.

## Rollback

- RMW: set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in `.env` (Fast DDS is still in the image).
- Camera: `ros2 launch turtlelima_bringup camera.launch.py transport_format:=png pointcloud:=false`.
- Server reading raw: `ros2 launch turtlelima_segmentation segmentation.launch.py input_transport:=raw compress_mask:=false`.

# semnav_ws — distributed semantic navigation testbed

Nav2 navigating with a semantic costmap layer, where **segmentation runs on a
GPU server** and the robot only carries the camera and the control loop.

Built as the measurement apparatus for the network-latency study: every stage
is timestamped, the link is impaired on purpose, and the thing that breaks
first is instrumented rather than hidden.

---

## The problem this structure solves

Nav2's semantic segmentation layer needs two inputs that must be **pixel-aligned
and time-synchronized**: a class mask, and a point cloud.

On one machine that is free — both come from the same camera callback. Split
across a network it stops being free, and the failure is silent:

```
t=0ms     camera captures frame F, cloud C(F)
t=0ms     F compressed, sent to server
t=120ms   server returns mask M(F)
t=120ms   robot has moved ~10cm.  Live cloud is now C(t=120)
```

Pair `M(F)` with `C(t=120)` and the semantic labels land on the wrong
geometry. Obstacles get painted where the robot *is*, not where they were
*seen*. Nothing errors. The costmap just becomes quietly wrong, and worse the
faster the robot moves.

**The fix:** buffer clouds on the robot keyed by timestamp, and when a mask
comes back, pair it with the cloud it was actually computed from. The pair
carries the original capture stamp, so Nav2's TF lookup resolves it to the
pose the robot held at capture time.

That is what `semnav_sync/cloud_resync_node` does, and it is also the
experiment's measurement point — it reports the age of every mask and counts
every pairing it could not make.

---

## Why Nav2 lives on the robot

Two partitions are possible. This workspace implements the second.

**A — everything on the server.** Robot sends images + depth + TF; server runs
segmentation, costmap, planner and controller; `/cmd_vel` comes back.
*Consequence:* the motion control loop closes over Wi-Fi. Downlink delay
directly destabilises the controller, and a link dropout means no brakes.

**B — Nav2 on the robot, segmentation on the server.** Robot sends colour
frames; server returns masks; the robot owns the costmap, planner and
controller.
*Consequence:* only semantic enrichment crosses the network. Degrade the link
and the robot loses its *semantics*, never its *reflexes*.

B is better on all three counts that matter here:

| | A | B |
|---|---|---|
| Control loop crosses network | yes | **no** |
| Uplink payload | colour + depth/cloud | colour only |
| Isolates segmentation latency as the variable | no — confounded with control delay | **yes** |

B also degrades gracefully, which is why the costmap below runs **two layers**:

- `voxel_layer` — fed by the **local** cloud. Never crosses the network, never
  goes stale. Knows *something is there*, not what it is.
- `semantic_layer` — fed by the **resynced** pair that made the round trip.
  This is the layer the experiment degrades.

Run with the semantic layer alone to see the raw effect of network
degradation. Run with both to see what a sane deployment would actually do.
The difference between those two curves is a result worth publishing.

---

## Topology

```
ROBOT (Jetson Orin Nano)                      SERVER (RTX 5070)
─────────────────────────                     ──────────────────
realsense2_camera
  ├─ /camera/color/image_raw
  │        │
  │        └─► republish ─► .../compressed ──────► segmentation_node
  │                                                   │  model inference
  │                                                   │  header passed through
  │           /segmentation/mask       ◄──────────────┤  UNCHANGED
  │           /segmentation/confidence ◄──────────────┤
  │           /segmentation/label_info ◄──────────────┘  (latched)
  │
  └─ /camera/depth/color/points
           │                    │
           │                    └──────────────┐
           ▼                                   ▼
     voxel_layer                        cloud_resync
     (geometry, always fresh)             buffers clouds by stamp,
           │                              pairs returning masks
           │                                   │
           │                            /semnav/{mask,points,confidence}
           │                                   │
           │                                   ▼
           └──────────────────────────►  semantic_layer
                                               │
                                        Nav2 costmap ─► planner ─► controller ─► /cmd_vel
```

**The one rule:** `segmentation_node` copies the input image header — stamp
*and* frame_id — onto the mask. The mask describes the instant the shutter
opened, not the instant inference finished. Restamping with "now" destroys
the only information the robot has for putting semantics back on the right
geometry, and the error stays invisible until something gets hit.

---

## Build

Both machines need the workspace. ROS 2 Jazzy assumed.

```bash
# the semantic layer plugin is third-party; vendor it in
cd src && git clone https://github.com/kiwicampus/semantic_segmentation_layer.git && cd ..

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Server also needs `onnxruntime-gpu` (or torch) and `opencv-python` — only for
the real backends, not for `stub`.

---

## Run

Same `ROS_DOMAIN_ID` on both machines, and **pin the DDS interface**:

```bash
# ROBOT
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/src/semnav_bringup/config/cyclonedds_robot.xml
ros2 launch semnav_bringup robot.launch.py buffer_duration_s:=2.0 jpeg_quality:=80

# SERVER
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/src/semnav_bringup/config/cyclonedds_server.xml
ros2 launch semnav_bringup server.launch.py backend:=stub
```

Edit `NetworkInterface name=` in both XML files to your real interfaces
(`ip -br link`). This is not optional: with a VPN present, Cyclone will pick
the virtual interface and your traffic will not traverse the link you are
shaping.

### Bring-up order

1. `backend:=stub`, no impairment. Confirm `cloud_resync` logs a high pairing
   rate and small mask age. **If pairing fails here, nothing downstream means
   anything** — it is a clock or timestamp problem, not a network one.
2. Swap in the real model. Mask age jumps by the inference time.
3. Send a Nav2 goal and confirm the robot avoids a semantically-marked object.
4. Only now start impairing the link.

---

## The knobs that are experimental factors

Not tuning parameters — independent variables. Sweep them, don't optimise them.

| Where | Parameter | What it trades |
|---|---|---|
| `cloud_resync` | `buffer_duration_s` | how much delay the robot can compensate before the semantic layer goes blind |
| `robot.launch.py` | `jpeg_quality` | uplink bandwidth against mask fidelity |
| `nav2_costmap.yaml` | `observation_persistence` | how long a stale observation is still trusted |
| `nav2_costmap.yaml` | `tile_map_decay_time` | how long semantic cost lingers after the mask stops arriving |
| `server.launch.py` | `max_input_age_ms` | drop-on-arrival vs. inferring on frames nobody can use |
| `netem.sh` | profile, **per direction** | the impairment itself |

### Impairment

```bash
sudo ./scripts/netem.sh wlan0 delay150   # on the ROBOT = uplink
sudo ./scripts/netem.sh eth0  delay50    # on the SERVER = downlink
sudo ./scripts/netem.sh wlan0 clear
```

Uplink and downlink are different mechanisms — stale perception versus delayed
actuation. Shaping only round-trip confounds them permanently.

### Measurement

```bash
ros2 topic echo /semnav/pairing_stats    # mask age, pairing hit/miss
ros2 topic echo /semnav/server_stamps    # uplink / inference / server total

ros2 bag record /semnav/pairing_stats /semnav/server_stamps \
                /tf /tf_static /odom /cmd_vel /local_costmap/costmap
```

`mask_age_ms` and `total_missed` are the dependent variables. The number worth
finding is the **delay at which pairing starts failing** — that is where
buffering stops compensating and the semantic layer goes blind. State the
result in milliseconds of staleness, not in properties of this lab's Wi-Fi,
and it transfers to any offloaded robot.

---

## Check your clock first

```bash
./scripts/check_clock.sh <peer-ip>
```

One-way delay cannot be recovered from round-trip time when the two
directions are impaired independently — which is exactly what this experiment
does. Every `uplink_ms` and `mask_age_ms` is only as trustworthy as the clock
offset is small. At 50 ms injected delay, an offset above ~5 ms makes the
measurement unusable. Run chrony or PTP between the machines, and record the
offset alongside each experiment block.

---

## Gotchas

- **Depth must be registered to colour.** If it isn't, mask and cloud describe
  different pixels and the layer is wrong before the network is even involved.
- **Class names must agree in three places**: `ontology.yaml`, the
  `class_names` parameter of `segmentation_node`, and the `classes:` lists in
  `nav2_costmap.yaml`.
- **`label_info` is latched** (transient-local). If the costmap starts before
  the server it still needs the ontology, which is why that QoS is not
  best-effort like the rest.
- **Never point `semantic_layer` at `/segmentation/mask` directly.** It must
  read the resynced `/semnav/*` topics. Wiring it to the raw topic is exactly
  the bug this architecture exists to prevent, and it will look like it works.
- **`publish_on_miss` stays False.** Publishing an unpaired mask puts
  obstacles in the wrong place, which is worse than having none.

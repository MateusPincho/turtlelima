# Implementation backlog

Ordered by what unblocks what, and by what would hurt most if discovered late.

**If you only do three things this week:** 0.1, 0.2, 0.6. Each is an hour or
two, and any one of them failing changes the design rather than the schedule.

**Minimum viable project** for the course: blocks 0, 1, 2, 4, 5. Block 3 is
cheap and runs in parallel on the server. Block 6 is the stretch.

---

## Block 0 — De-risk before building anything

Every task here is cheap now and expensive later. Do them all before writing
integration code.

- [ ] **0.1 — Confirm depth is registered to colour.**
  Launch the camera, echo `/camera/depth/color/points`, verify the cloud is in
  the colour optical frame with `align_depth.enable` on. Put a box at a known
  distance and check it lands where the colour image says it is.
  *If this is wrong, mask and cloud describe different pixels and the semantic
  layer is broken before the network is involved.*

- [ ] **0.2 — Verify the server can run a model at all.**
  Fresh env. `torch` from the cu128 index, or `onnxruntime-gpu`. Confirm
  `torch.cuda.get_arch_list()` contains `sm_120`, then run one forward pass.
  *The 5070 is Blackwell. Anything built before torch 2.7 dies with "no kernel
  image is available for execution on the device", and no amount of
  integration work routes around it.*

- [ ] **0.3 — Build `semantic_segmentation_layer` on Jazzy.**
  It is third-party and may be pinned to an older Nav2. If it will not build,
  decide **now** between patching it and the fallback: project non-ground
  points yourself and feed a stock `obstacle_layer` as a PointCloud2
  observation source. The fallback loses per-class costs but keeps the
  experiment intact.

- [ ] **0.4 — Measure Jetson headroom with no server in the loop.**
  Camera + JPEG compression + full Nav2 + a dummy goal. Watch CPU, GPU and
  whether the local costmap holds its update rate.
  *If the Jetson is already saturated, repartition before you build on it.*

- [ ] **0.5 — Characterise the lab Wi-Fi, unloaded and then with the camera
  stream running.**
  RTT, jitter, loss, throughput. If the idle link is already unstable you need
  a dedicated AP, or you will be measuring the lab's network rather than your
  independent variable.

- [ ] **0.6 — Get chrony running between Jetson and server, and record the
  offset.**
  *Every latency number in the project is only as trustworthy as this. At
  50 ms injected delay an offset above ~5 ms makes the measurement unusable.*

---

## Block 1 — Stand up the testbed (objective 1)

- [ ] **1.1** Build the workspace on both machines. Pin `NetworkInterface` in
  both CycloneDDS configs to the real interfaces. Confirm discovery with
  `ros2 topic list` from each side.
  *Check `ip -br link` first — a VPN interface will silently win otherwise.*

- [ ] **1.2** End-to-end with `backend:=stub`, no impairment.
  **Done when** `cloud_resync` reports pairing > 95% and mask age ≈ uplink +
  stub time. *If pairing fails on a clean link, it is a clock or timestamp
  bug, not a network one. Fix it here.*

- [ ] **1.3** Pick the segmentation model, export to ONNX or TorchScript, wire
  it into the backend. Check masks on real images from your corridor, not on
  ADE20K samples.

- [ ] **1.4** Define the class ontology for your actual lab environment.
  Make `ontology.yaml`, the `class_names` parameter, and the `classes:` lists
  in `nav2_costmap.yaml` agree. *Three places, one source of truth.*

- [ ] **1.5** Send a Nav2 goal with the semantic layer active and confirm the
  robot avoids a semantically-marked object it would otherwise drive through.

> **Gate 1 — semantic avoidance works on a clean network.** Nothing after this
> point is meaningful until this demo runs reliably.

---

## Block 2 — Instrumentation and nominal baseline

- [ ] **2.1** Write the analysis script: rosbag → DataFrame → per-stage latency
  table. Build it now, while you have clean data to test it on.

- [ ] **2.2** Define the fixed test route. Mark the start pose on the floor so
  runs are actually repeatable, and fix the goal pose.

- [ ] **2.3** Decide how you detect collisions and near-misses. Create3 bumper
  events for contact; LiDAR minimum distance as continuous ground truth.
  *The LiDAR earns its place here — not in the navigation stack, but as the
  independent measurement the camera pipeline is scored against.*

- [ ] **2.4** Run N nominal missions. This is your **N0 control**, and since
  the system has no published benchmark, it is the only baseline you get.

> **Gate 2 — a per-stage latency budget on a clean link, with variance.**

---

## Block 3 — Offline mask degradation (Phase A)

Needs no robot after 3.1. Run it on the server in parallel with block 1.

- [ ] **3.1** Record a rosbag of raw colour + cloud along the test route.
- [ ] **3.2** Sweep JPEG quality and resolution offline; run segmentation on
  each; compare against the full-quality mask as reference.
  *Reference, not ground truth — you are measuring degradation relative to
  nominal, which is the actual question and needs no hand-labelling.*
- [ ] **3.3** Implement **false free space** and **false obstacle**
  separately, computed in projected BEV area rather than image pixels.
  *IoU averages the two and hides the one that causes collisions. The
  homography also amplifies error with distance, so pixel error does not tell
  you the safety consequence.*
- [ ] **3.4** Produce the bandwidth ↔ fidelity curve.

> **Gate 3 — you can name the compression level at which false free space
> becomes unacceptable, in square metres inside stopping distance.**

---

## Block 4 — Online degradation (objective 2)

- [ ] **4.1** Validate the `netem` profiles produce the one-way delay you think
  they do. Measure it; do not assume it.
- [ ] **4.2** Uplink sweep. Pairing rate and mask age vs injected delay.
- [ ] **4.3** **Find the buffer cliff** — the delay at which pairing starts
  failing, as a function of `buffer_duration_s`.
  *This is the headline result: the point where latency compensation stops
  compensating and the semantic layer goes blind.*
- [ ] **4.4** Downlink sweep. Path-tracking error, command oscillation, route
  completion time.
- [ ] **4.5** Semantic-layer-only vs both-layers, under the same impairments.
  *The gap between those two curves is what a real deployment buys by keeping
  a geometric layer local.*
- [ ] **4.6** Repeat the key conditions enough times to report variance rather
  than single runs.

> **Gate 4 — dose–response curves for both directions, with the staleness
> threshold stated in milliseconds.**

---

## Block 5 — Analysis and report

- [ ] **5.1** Latency budget per impairment regime; name the dominant stage in
  each.
- [ ] **5.2** Test the hypotheses explicitly, including the one that predicts
  safety degrades before task success.
- [ ] **5.3** Write the report. State results in **staleness**, not in
  properties of this lab's Wi-Fi, so they transfer.
- [ ] **5.4** Package dataset, impairment profiles and code.

---

## Block 6 — VLN layer (objective 3, stretch)

- [ ] **6.1** Offline bake-off: run candidate local models over recorded frames
  and compare waypoints against the cloud model's recorded responses.
  *No robot, no network — the cheapest way to answer whether anything you can
  host on 12 GB points well enough.*
- [ ] **6.2** Integrate the chosen model as the waypoint source.
- [ ] **6.3** Mission-level runs, cloud vs local inference: duration, stopped
  time, completion.
- [ ] **6.4** Measure plan age at the moment motion resumes.
  *Cognitive latency is a pure time cost only in a static scene. In a corridor
  with people in it, a stale plan is a safety cost too — that conditional is
  worth testing rather than assuming.*

---

## Parking lot

Things that will come up and are easier to decide before they bite.

- [ ] Reliable vs best-effort QoS on the image stream under loss — retransmit
  adds latency, drop loses frames. Worth a measured comparison; it is a
  networks result in its own right.
- [ ] Docker images for both machines, so the environment is part of the
  experimental record.
- [ ] Gazebo variant of the whole topology for dense sweeps without robot time.
  Check first that segmentation behaves sanely on synthetic imagery — if the
  mask falls apart in sim, the reactive path is unrepresentative and the idea
  dies.
- [ ] Decide whether `observation_persistence` and `tile_map_decay_time` are
  held fixed or swept. They interact with `buffer_duration_s`, and sweeping
  all three is a bigger grid than it looks.
- [ ] Power and thermal behaviour of the Jetson over long run blocks —
  throttling mid-sweep would be a confound you would not see in the data.

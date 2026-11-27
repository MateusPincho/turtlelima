# Network benchmark: robot (Jetson) ↔ segmentation server

Methodology step for TASKS.md **0.5** (characterise the lab Wi-Fi, idle and with
the camera stream) and **0.6** (clock offset between the Jetson and the server).
It runs **before** the `semantic_segmentation_layer` is used in navigation. It
gives the nominal (N0) network baseline, so the later netem sweeps measure the
delay we inject and not the delay the lab Wi-Fi already adds.

Scripts: [`benchmark/network/`](../benchmark/network/). Created 2026-11-26.

---

## 1. What the network has to carry

Partition B of SPECIFICATION_1.md puts Nav2 on the robot and segmentation on the
server. Only the semantic data crosses the link:

| Direction | Topic | Message | Size | Rate | Load |
|---|---|---|---|---|---|
| uplink (robot → server) | `/camera/realsense/color/image_raw/compressed` | JPEG q80, 640×480 | ~50 KB | 15 Hz | ~8 Mbit/s |
| downlink (server → robot) | `/mask` | raw mono8 class ids | 307 KB | ≤ 15 Hz | ~37 Mbit/s |
| downlink | `/confidence` | raw mono8 0–255 | 307 KB | ≤ 15 Hz | ~37 Mbit/s |
| downlink | `/label_info` | LabelInfo, latched | tiny | once | – |

Every topic is best effort with depth 1 (sensor data QoS). Each 307 KB mask is
split by CycloneDDS into about 220 UDP datagrams. If one of them is lost, the
whole mask is lost, because best effort doesn't retransmit. So a small packet
loss rate turns into a much larger message loss rate.

### The time limits that come from the code

These numbers come from the layer source and `nav2_params.yaml`. The benchmark
checks the network against them:

| Limit | Value | Where it comes from |
|---|---|---|
| **Staleness budget** | ~667 ms | The layer keeps `sensor_data_depth` = 10 clouds in its sync queue (`semantic_segmentation_layer.cpp:78`). At 15 Hz that is 0.667 s of history. A mask that comes back later than that finds no matching cloud and is dropped without any error. |
| Sync slop | 100 ms | `setMaxIntervalDuration(0.1 s)` of the ApproximateTime sync (`semantic_segmentation_layer.cpp:241,274`). This is between mask and cloud *stamps*, not arrival times. The segmentation node copies the image header, so it is not consumed by network delay. It becomes a problem only if the server restamps. |
| Controller period | 50 ms | `controller_frequency: 20.0` |
| Local costmap update | 200 ms | `local_costmap.update_frequency: 5.0` |
| Costmap timeout | 300 ms | `controller_server.costmap_update_timeout: 0.30` |
| Semantic persistence | 5 s | `tile_map_decay_time: 5.0`: how long semantic cost stays on a tile after masks stop arriving |
| Clock offset | < 5 ms | TASKS.md 0.6: above this the one-way numbers at 50 ms of injected delay are unusable |

---

## 2. Metrics and why each one matters

| Metric | Definition | Measured with | What it does to navigation and control |
|---|---|---|---|
| **RTT** min/p50/p95/p99/max | send → reply, ms | `ping`, `ros_probe.py` | Sets the base age of every mask. p99 has to stay well under the 667 ms staleness budget. |
| **One-way delay** up / down | uplink = server rx − robot tx, downlink = robot rx − server tx | `ros_probe.py` + chrony | Uplink delay makes the perception stale. Downlink delay delays the semantics on the costmap (SPEC: "stale perception vs delayed actuation"). The two have to be measured separately, because netem impairs each direction separately. |
| **Jitter** | (a) `ping` mdev (std of RTT); (b) RFC 3550 IPDV, `J += (│Dᵢ−Dᵢ₋₁│ − J)/16` | `ping`, `iperf3` UDP, `ros_probe.py` | Variable mask age makes the semantic cost on the costmap flicker, and it makes the latency hard to compensate. A constant clock offset cancels out of one-way IPDV, so it is valid without clock sync. |
| **Packet loss** and **burst loss** | lost %, number and longest run of consecutive losses | `ping -O`, `iperf3` UDP | Bursts matter more than the average: a burst longer than the queue means the semantic layer goes blind. |
| **Message loss** (DDS) | ping messages with no reply, split into uplink and downlink using the echo log | `ros_probe.py` | The loss the costmap actually sees, after fragmentation. |
| **Throughput / capacity** | TCP goodput per direction, UDP rate sweep up to the knee where loss starts | `iperf3` | The downlink has to carry about 74 Mbit/s. The headroom (capacity ÷ demand) tells how close the link is to saturation. |
| **Latency under load** (bufferbloat) | ping RTT while iperf3 saturates the link | `ping` + `iperf3` | The real downlink traffic itself can fill the AP queues and delay everything, including TF/odom if they share the link. |
| **Reordering / duplicates** | late or repeated replies | `ros_probe.py`, `ping`, `iperf3` | ApproximateTime assumes stamps arrive in order. |
| **Clock offset** | chrony `System time`/`RMS offset`, and an NTP-style estimate from the fastest 10 % of small-message round trips | `chronyc`, `ros_probe.py` | Without it, uplink/downlink and server-side ages can't be trusted. |
| **Wi-Fi state** | RSSI, MCS/bitrate, MAC retries and failures during the run | `iw station dump` | Explains loss and jitter, and lets runs on different days be compared. |
| **Path MTU** | 1472 B payload with DF must pass | `ping -M do` | A smaller MTU (VPN, tunnel) multiplies the number of fragments per mask. |
| **Real topic rate, gaps and age** | arrival rate, inter-arrival distribution, missed frames, `age = rx − header.stamp` | `topic_monitor.py` | On the robot, `/mask` age is the whole capture → uplink → inference → downlink latency, and it needs **no** clock sync (the stamp comes from the robot's own camera). |
| **DDS software floor** | same probe with the echo on the Jetson itself | `ros_probe.py --prefix /netbench_local` | Python, serialisation and executor cost. This is subtracted mentally from the network numbers. |

---

## 3. Tools to install

| Tool | Package (Ubuntu) | Where | Used for |
|---|---|---|---|
| `ping` | `iputils-ping` | host | RTT, mdev jitter, loss, burst loss, PMTU |
| `iperf3` | `iperf3` | host (Jetson and server) | TCP capacity, UDP jitter and loss, rate sweep, load generator |
| `mtr` | `mtr-tiny` | host | per-hop loss and latency |
| `chronyd`/`chronyc` | `chrony` | **host only** (a container can't set the system clock) | clock synchronisation and offset |
| `iw` | `iw` | Jetson host | Wi-Fi RSSI, bitrate, retries |
| `ethtool` | `ethtool` | server host | wired link speed and duplex |
| `tc`, `ip`, `ss` | `iproute2` | host | interface info (later also netem) |
| `jq` | `jq` | anywhere | inspect the JSON outputs by hand |
| `fping` | `fping` | optional | ping cross-check |
| `python3` | `python3` | anywhere | `analyze.py` (standard library only) |
| `matplotlib` | `python3-matplotlib` | optional | plots in `analyze.py` |
| `rclpy`, `rmw_cyclonedds_cpp` | already in the `turtlelima` images | **container** | `ros_probe.py`, `topic_monitor.py` |

Install them on **both** hosts:

```bash
cd src/turtlelima/benchmark/network
sudo ./install_tools.sh          # apt (or pacman); disables the Debian iperf3 daemon if it was started
./install_tools.sh --check       # lists what is present/missing, with versions
```

The ROS part needs no installation: run it inside `docker compose exec dev bash`
(or `dev-gpu` on the server), where the workspace is mounted at
`/turtlelima/src`. If you prefer to run the whole sequence from one shell inside
the container, run `sudo ./install_tools.sh` there too, but it has to be
repeated every time the container is recreated. Chrony still has to run on the host.

---

## 4. One-time setup

1. **Fill in `benchmark/network/config.env`** (same file on both machines):
   `ROBOT_IP`, `SERVER_IP` (the addresses on the link the ROS traffic uses,
   not Tailscale), `ROBOT_IFACE` (e.g. `wlan0`), `SERVER_IFACE` (e.g. `eth0`).
   The rates and payload sizes already match the real pipeline.
2. **Clock sync** (TASKS 0.6). The server is the reference and the Jetson follows it:
   ```bash
   sudo ./chrony_setup.sh server     # on the server
   sudo ./chrony_setup.sh client     # on the Jetson; wait 1-2 min
   chronyc tracking                  # Jetson: 'System time' and 'RMS offset' well below 1 ms
   ```
3. **Socket buffers and DDS interface**, as in `implementation.md` ("Host
   setup"): `net.core.rmem_max`, `CYCLONEDDS_RCVBUF=10MB`, and the
   `NetworkInterface` override if a VPN or several interfaces are present. Use the
   same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on both
   machines. The probe uses the same middleware settings as the real pipeline,
   so a wrong setting here shows up in the probe numbers too.
4. Check that each side sees the other: `ros2 topic list` on both, `ping $SERVER_IP` from the Jetson.

---

## 5. Procedure

Run two conditions. Each one takes about 12 minutes.

| Label | State of the system |
|---|---|
| `idle` | camera, segmentation and Nav2 **stopped**. This is the raw link. |
| `camera_load` | `camera.launch.py` on the robot and `segmentation.launch.py` on the server **running**. This is the link carrying the real 82 Mbit/s, and the real topics are monitored. |

Repeat each condition at least 3 times, at different times of day (the lab
Wi-Fi is shared). That gives variance and not a single sample (TASKS 4.6).
Keep the robot parked at the same spot, and note the spot and the AP distance.

### 5.1 On the server

```bash
# host shell (iperf3 + chrony log)
./server_run.sh --label idle --stage net
# container shell (DDS echo; for camera_load also add --monitor)
./server_run.sh --label idle --stage ros
```

If the host has both iperf3 and rclpy, run just `./server_run.sh --label idle`.
Leave it running and stop it with Ctrl-C after the Jetson finishes.

### 5.2 On the Jetson

```bash
# host shell: IP-level tests (~7 min)
./jetson_run.sh --label idle --stage net
# container shell: DDS-level tests (~5 min)
./jetson_run.sh --label idle --stage ros
```

`--quick` does a 3-minute smoke test (shorter runs, no rate sweeps). Use it once
first, to check the addresses and discovery.

### 5.3 Collect and analyse

```bash
# copy the server's folder next to the Jetson's, same label and date
scp -r server:<ws>/src/turtlelima/benchmark/network/results/<date>_idle/server \
       results/<date>_idle/
python3 analyze.py results/<date>_idle results/<date>_camera_load
```

Each folder gets a `results.md` (PASS/FAIL checks, all the tables, plots), and
`results/comparison.md` puts the key metrics of the two conditions side by side.

### 5.4 What each step runs

`jetson_run.sh --stage net`:

| # | Test | Command (summary) | Output |
|---|---|---|---|
| 0 | host, link, clock snapshot | `ip`, `ethtool`, `tc qdisc`, `sysctl`, `iw station dump`, `chronyc tracking` (before and after) | `host_info.txt`, `before_*`, `after_*` |
| 1 | reachability | `ping -c 3`, TCP connect to the iperf3 port | `ping_sanity.txt` |
| 2 | RTT/jitter/loss | `ping -D -O -i 0.02 -c 3000` (60 s at 50 Hz, faster than the 20 Hz controller) | `ping_baseline.txt` |
| 3 | PMTU | `ping -M do -s 1472` / `-s 1473` | `pmtu_*.txt` |
| 4 | path | `mtr -n -c 30 --json` | `mtr.json` |
| 5 | TCP capacity | `iperf3 -t 30` up, then `-R` down | `iperf_tcp_*.json` |
| 6 | UDP at the operating point | `-u -b 8M` up, `-u -b 80M -R` down, `-l 1400` | `iperf_udp_*.json` |
| 6b | UDP rate sweep | up 4/8/16/32 Mbit/s, down 40/80/120/160/200 Mbit/s, 10 s each | `sweep_udp_*.json` |
| 7 | latency under load | ping during UDP-down at 80 Mbit/s, TCP-down, TCP-up | `ping_load_*.txt`, `load_*.json` |

`jetson_run.sh --stage ros`:

| # | Test | Payload up / down | QoS | Output |
|---|---|---|---|---|
| 1 | software floor (echo on the Jetson) | 50 KB / 307 KB | best effort | `probe_<label>-local_best_effort.*`, `local/` |
| 2 | clock offset estimate | 64 B / 64 B | best effort | `probe_<label>-small_best_effort.*` |
| 3 | real sizes, as the pipeline | 50 KB / 307 KB | best effort | `probe_<label>-real_best_effort.*` |
| 3b | real sizes, retransmitting | 50 KB / 307 KB | reliable, depth 1 | `probe_<label>-real_reliable.*` |
| 4 | real topics (not on `idle`) | `/mask`, `/confidence`, compressed colour, points | best effort | `monitor_*.csv` |

How the probe works: the Jetson publishes `sensor_msgs/Image` at 15 Hz on
`/netbench/<qos>/ping`, with the send time in `header.stamp` and `run:seq` in
`frame_id`. The server echo answers on `/netbench/<qos>/pong`, padded to the
reply size. Its receive and send times go in `frame_id`. Each message
therefore yields four timestamps, giving RTT, uplink, downlink and server hold
time. The server also logs every ping it receives (`echo_*.csv`), so a lost
message is attributed to the uplink or the downlink.

---

## 6. Results

### 6.1 Tool validation (loopback, done 2026-11-26)

Before going to the robot, the whole chain (`server_run.sh`, `jetson_run.sh
--quick`, `analyze.py`) was run on one machine over `lo`. The machine was the
development laptop (`ros:jazzy-ros-base` container, CycloneDDS, 15 Hz,
20 s per probe run). This checks that every script runs and that every output
parses. The numbers also give the **software floor** of the probe without any
network:

| Run (loopback) | payload up / down | RTT p50 | RTT p99 | RTT max | loss | IPDV (RTT) |
|---|---|---|---|---|---|---|
| DDS best effort | 50 KB / 307 KB | 0.83 ms | 1.33 ms | 1.56 ms | 0 / 300 | 0.10 ms |
| DDS reliable | 50 KB / 307 KB | 0.80 ms | 1.13 ms | 1.17 ms | 0 / 300 | 0.08 ms |
| DDS best effort | 64 B / 64 B | 0.49 ms | 0.76 ms | 0.86 ms | 0 / 300 | 0.12 ms |
| ICMP | 64 B | 0.02 ms | 0.08 ms | 0.10 ms | 0 / 500 | 0.003 ms |

The passive monitor was also checked against a synthetic `/mask` publisher with
80–150 ms of delay and 5 % drop. It reported 14.0 Hz, 6.5 % missed frames,
an age p50 of 134 ms, and 91.9 % of colour frames returned as masks. That
matches what was injected (plus up to one 66 ms tick of quantisation from the
fake publisher).

**Takeaway:** the probe adds about 1 ms of its own (Python + serialisation on a
laptop CPU). That is two orders of magnitude below the 667 ms budget, so the
probe can resolve the effects that matter. The Jetson's floor will be higher,
which is why step `ros/1` measures it on the Jetson itself in every run.

### 6.2 Lab link: idle

Run 1: 2026-11-27, ~10:09 local time (-03). Source: `benchmark/network/results/20261127_idle/results.md`.

**Conditions of this run** (they differ from the assumptions in §3–§4):

- Robot: Jetson `turtlelima` (192.168.0.40) on Wi-Fi `wlP1p1s0`, SSID `LIMA`, **2.4 GHz (2427 MHz, channel 4)**, 2×2 VHT, Wi-Fi **power save on** (left as found). Parked at the bench, close to the AP (RSSI about −40 dBm).
- Server: `galego-ws` (192.168.0.106) is an **LXC container** on a wired `eth0` (RTX 5070). ROS runs inside the Docker container `turtlelima-dev-gpu` on both sides (network = host).
- RMW: **Cyclone DDS on both sides**. The server image only had Fast DDS, so `ros-jazzy-rmw-cyclonedds-cpp` and `docker/cyclonedds.xml` (as `/opt/dds/cyclonedds.xml`) were installed by hand into the running container. This does not survive a container rebuild. `CYCLONEDDS_URI` pins `eth0` on the server and `wlP1p1s0` on the robot. `CYCLONEDDS_RCVBUF=default`, and `net.core.rmem_max` is at the kernel default (212992) on the Jetson.
- **No chrony** (decided for this run). One-way delays are corrected only with the probe's own offset estimate, so they are indicative. RTT, loss, jitter and `/mask` age on the robot don't depend on it.
- `iw station dump` returns nothing with this Wi-Fi driver, so the MAC retry rate is not available.

| Metric | Run 1 | Run 2 | Run 3 | Mean ± std |
|---|---|---|---|---|
| Wi-Fi: RSSI (dBm) / tx bitrate / MAC retry % | −40 / 173.3 Mbit/s (VHT-MCS 8, 2 SS) / n/a | | | |
| Clock offset, chrony / probe estimate (ms) | n/a / 38.4 (drifting: 24.5 ten minutes earlier) | | | |
| ICMP RTT p50 / p95 / p99 / max (ms) | 1.24 / 5.57 / 8.50 / 56.9 | | | |
| ICMP mdev / IPDV jitter (ms) | 3.33 / 1.85 | | | |
| ICMP loss % / longest burst | 0 / 0 (3000 probes) | | | |
| TCP capacity up / down (Mbit/s) | 114.4 / 136.6 | | | |
| UDP @ 8 Mbit/s up: jitter (ms) / loss % | 0.56 / 0 | | | |
| UDP @ 80 Mbit/s down: jitter (ms) / loss % | 0.17 / 0 | | | |
| UDP downlink knee (highest rate with < 1 % loss) | 120 Mbit/s (160 Mbit/s: 141.5 achieved, 9.0 % loss) | | | |
| p95 RTT under TCP-down load − idle p95 (ms) | +230.4 (TCP-up: +48.9, UDP-down 80 Mbit/s: +3.5) | | | |
| DDS floor on Jetson, RTT p50 / p99 (ms) | 4.06 / 6.46 | | | |
| DDS BE real sizes: RTT p50 / p99 (ms) | 22.99 / 37.21 | | | |
| DDS BE real sizes: uplink / downlink p50 (ms) | 5.07 / 17.80 (offset-corrected, unsynchronised) | | | |
| DDS BE: loss % (up / down) / longest burst | 0 / 0 / 0 (900 messages) | | | |
| DDS REL real sizes: RTT p50 / p99 (ms), loss % | 23.26 / 43.18, 0 | | | |

### 6.3 Lab link: carrying the camera pipeline

Run 1: 2026-11-27, ~10:23 local time. `camera.launch.py` was running on the Jetson and `segmentation.launch.py` (SegFormer b2, TensorRT EP) on the server, both on Cyclone. Source: `results/20261127_camera_load/results.md`, plus `results/20261127_camera_load_fresh/results.md` for the robot `/mask` rows.

The segmentation node **stalled** during the run (see §6.5). By the time the robot-side monitor ran (step ros/4, ~14 min after launch), no `/mask` was being published, so that monitor recorded 0 masks. The `/mask` rows below come from a 45 s capture taken right after a fresh restart of the segmentation launch, with the camera still running (`results/20261127_camera_load_fresh`).

| Metric | Run 1 | Run 2 | Run 3 | Mean ± std |
|---|---|---|---|---|
| ICMP RTT p50 / p99 (ms), loss % | 1.32 / 16.48, 0 | | | |
| DDS BE RTT p50 / p99 (ms), loss % | 25.87 / 39.93, 0 | | | |
| `/mask` on robot: rate (Hz) / missed frames % | 5.07 / 26.4 | | | |
| `/mask` age p50 / p95 / p99 / max (ms) | 329.5 / 389.5 / 404.2 / 425.0 | | | |
| `/mask` inter-arrival p95 / max (ms) | 376.8 / 787.1 | | | |
| colour frames that came back as a mask (%) | 34.0 (227 / 668) | | | |
| compressed colour at server: rate (Hz), age p50 (ms) (= uplink + decode, needs sync) | 14.88, 133.7 raw (unsynchronised; the same topic is already 122.6 ms old on the robot itself) | | | |
| Mbit/s measured: colour up / mask + confidence down | 4.06 (33 KB/frame) / 25.2 (12.5 + 12.6) | | | |

Other values from this run: the DDS floor on the Jetson rises to 6.13 / 8.64 ms (p50 / p99) with the camera running (CPU contention). On the server, `/segmentation/image_raw` (the JPEG decoder output) runs at only 10.5 Hz with 29 % missed frames, and the segmentation node uses 730–810 % CPU. The robot's local point cloud (4.8 MB, 532 Mbit/s over loopback) arrives at 13.5 Hz with 9 % missed frames in a Python subscriber.

### 6.4 Pass/fail against the navigation limits

`analyze.py` evaluates these checks automatically:

| Check | Threshold | If it fails |
|---|---|---|
| Clock offset | < 5 ms | Fix chrony before trusting any one-way number. The RTT numbers are still valid. |
| ICMP loss (idle) | < 1 % | The link itself is unstable. Get a dedicated AP or channel, or you will be measuring the lab and not the experimental factor (TASKS 0.5). |
| ICMP p99 RTT | < 50 ms (one control period) | Wi-Fi power save, a congested channel, or roaming. Check `iw` and disable power save on the Jetson (`iw dev wlan0 set power_save off`). |
| Path MTU | 1500 | A tunnel or VPN is in the path. Pin CycloneDDS to the real interface. |
| TCP downlink capacity | > 1.5 × 74 Mbit/s | Not enough headroom for raw masks. Drop `/confidence`, compress the mask (PNG is lossless), or lower the resolution or rate. |
| UDP downlink loss @ 80 Mbit/s | < 1 % | Same as above. Also check `CYCLONEDDS_RCVBUF`/`rmem_max`. |
| Wi-Fi MAC retry rate | < 10 % | Interference or weak signal. Move the AP or change the channel. |
| DDS BE message loss (real sizes) | < 1 % | Fragment loss is amplified. Compare with ICMP/UDP loss to tell the network from the DDS configuration. |
| DDS p99 RTT | < 667 ms | Masks will start missing the cloud queue. Raise `sensor_data_depth` or reduce the delay. |
| `/mask` p99 age | < 667 ms | The same, measured on the real pipeline. |

Leave the remaining numbers as measured values, without a pass/fail. They are
the N0 baseline the later netem sweeps (Block 4) are compared against: the
injected delay at which pairing starts failing is stated *relative to* these
numbers.

### 6.5 Findings of Run 1 (2026-11-27)

1. **The link is not the limit at today's load.** Both conditions pass every network check (no ICMP, UDP or DDS loss, ICMP p99 below 17 ms, DDS p99 below 40 ms, TCP down 129–137 Mbit/s, UDP knee at 120 Mbit/s). The DDS downlink p50 of ~18 ms for a 307 KB message is mostly airtime: 307 KB × 8 / ~140 Mbit/s ≈ 17.5 ms. Raw masks make the downlink the expensive direction.
2. **The segmentation server is the bottleneck.** It produces 5–6 masks/s for a 15 Hz camera, so only about a third of the frames come back. Its CPU use is about 800 %, and the two `nvidia-smi` samples showed 0 % GPU. The JPEG decoder alone drops to 10.5 Hz. The end-to-end `/mask` age is ~330 ms p50 and ~404 ms p99, so **about 60 % of the 667 ms staleness budget is used before any netem delay**. The real downlink load is ~25 Mbit/s, not the planned 74 Mbit/s.
3. **The segmentation node stalls on Cyclone.** After a few minutes (and once within the first minute after launch), the node stays in lifecycle state `active` and keeps receiving `/segmentation/image_raw` at 14.9 Hz, but it uses 0 % CPU and publishes no `/mask`. It uses `MultiThreadedExecutor` with the default callback group. This has to be fixed and checked before Block 4, or any long run will look like 100 % mask loss.
4. **Bufferbloat under TCP.** A saturating TCP download adds ~230 ms to the p95 RTT (and a TCP upload adds ~49 ms). The real pipeline is UDP best effort and adds only ~3.5 ms at 80 Mbit/s. But any bulk TCP flow on the same AP (scp, rosbag copy, reliable large topics) would push the mask age toward the budget.
5. **The clocks drift apart without sync.** The probe's offset estimate grew from 24.5 to 38.4 to 49.9 ms over ~30 min. The LXC server has no NTP service of its own (it follows its host). Chrony (§4 step 2) is needed before one-way numbers can be used.
6. Wi-Fi: 2.4 GHz channel 4 with power save on. Re-measure with power save off and, if possible, on 5 GHz. MAC retry counters are not readable with this driver.

## 7. Limits of the method

- **ICMP and iperf3 measure IP, not DDS.** A 1400 B UDP datagram is not a 307 KB
  fragmented message. Both are reported, and the gap between them shows what
  the middleware costs.
- **The probe is a Python node.** Its own overhead is measured (the software
  floor) and should be subtracted mentally. On the Jetson, check that the CPU is not
  saturated during the run (`tegrastats`), or the floor rises.
- **One-way delays rely on chrony.** The probe's own offset estimate assumes the
  tiny messages travel equally fast in both directions. Use it as a
  cross-check, not as a replacement.
- **Reliable with depth 1** behaves like the camera's reliable mode: if a sample
  is not acknowledged before the next one is written, the new sample replaces
  it. A loss in that mode means "overwritten", not "dropped by the network".
- **Wi-Fi is not stationary.** Single runs mean little, so report the spread over
  repeated runs and record RSSI, channel and time of day with each block.
- The results are stated in milliseconds of staleness and loss %, not as
  "the lab Wi-Fi is good". That way they carry over to the netem study and to
  other robots (SPEC: "state the result in milliseconds of staleness").

## 8. Files

| File | Purpose |
|---|---|
| `benchmark/network/config.env` | addresses, interfaces, durations, rates, payload sizes |
| `benchmark/network/install_tools.sh` | install/check the tools |
| `benchmark/network/chrony_setup.sh` | server = time reference, Jetson = client |
| `benchmark/network/common.sh` | shared shell helpers (logging, snapshots) |
| `benchmark/network/server_run.sh` | iperf3 server, chrony time series, DDS echo, optional topic monitor |
| `benchmark/network/jetson_run.sh` | runs the whole test sequence |
| `benchmark/network/ros_probe.py` | DDS ping/echo probe |
| `benchmark/network/topic_monitor.py` | passive monitor of the real topics |
| `benchmark/network/analyze.py` | raw output → `results.md`, `comparison.md`, plots |

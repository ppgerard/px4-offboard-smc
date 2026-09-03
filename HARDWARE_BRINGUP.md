# T2 indoor bring-up (no GNSS)

Clone, build, launch. Everything below has been exercised in SITL; **nothing in
it has flown on the real aircraft**, so treat every step as a test rather than a
procedure.

## 1. Workspace

```bash
mkdir -p ~/t2_ws/src && cd ~/t2_ws/src
git clone -b feature/t2-hardware-bringup git@github.com:ppgerard/px4-offboard-smc.git
git clone git@github.com:ppgerard/apriltag_ros_enhanced.git
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs && git checkout 56f8019425a16af2941df5a9288834398681e394 && cd ../..

cp -r ~/<OLD WORKSPACE>/src/camera_ros ~/t2_ws/src/       # see below

source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**Do NOT source the old workspace.** It contains its own `px4_msgs` (release/1.15,
which predates message versioning), and building or running with two different
`px4_msgs` on `AMENT_PREFIX_PATH` is how you get code compiled against one struct
layout loading the shared library of another. The layouts genuinely differ, so
the failure is silent garbage in message fields rather than a link error. If you
already built with it sourced, throw the build away and redo it in a clean shell:

```bash
cd ~/t2_ws && rm -rf build install log
```

Verified from scratch: the three repos build clean in ~4 min on a desktop, so
allow 10-15 on the Pi. `t2_indoor.repos` describes the same set if you prefer
`vcs import src < src/px4-offboard-smc/t2_indoor.repos`, but plain clones are
listed first because `vcs` is not installed everywhere.

**Copy `camera_ros` in; leave `libcamera` alone.** camera_ros is a normal ament
package and finds libcamera through pkg-config
(`pkg_check_modules(libcamera REQUIRED libcamera>=0.1)`), not through the
workspace, so rebuilding it here produces an equivalent binary. Check what it will
link against BEFORE building:

```bash
pkg-config --modversion libcamera        # expect 0.5.2 (the Pi's rpt build)
pkg-config --variable=prefix libcamera
```

If that resolves to something else, fix `PKG_CONFIG_PATH` first -- silently
relinking camera_ros against upstream libcamera loses Raspberry Pi camera support.

Copying the **libcamera source** into `src/` does nothing: it has only
`meson.build`, and colcon identifies packages by `package.xml` / `CMakeLists.txt`
/ `setup.py`, so it is ignored outright. It has to stay a meson install.

Do not `apt install ros-jazzy-camera-ros ros-jazzy-libcamera` as a shortcut: the
apt libcamera is upstream, and a Raspberry Pi camera needs the Pi's fork.

**Everything else comes from apt**, so no overlay is needed: `apriltag`,
`apriltag_msgs`, `image_proc`, `camera_info_manager`, `cv_bridge`,
`image_transport`, `image_view` are all in `/opt/ros/jazzy`.

**Watch out for the apt `ros-jazzy-apriltag-ros`.** It has the same package name
as `apriltag_ros_enhanced` and may already be installed. The workspace wins once
sourced, but if you forget to `source install/setup.bash` you will silently get
the apt one -- which has no `platform` frame support at all. Confirm with
`ros2 pkg prefix apriltag_ros`; it must point at your workspace.

## 2. Before anything is armed

**Topic names.** The `_v<N>` suffix comes from the *firmware's* message version
and is absent when that version is 0, so it differs between PX4 releases:

```bash
ros2 topic list | grep fmu/out
```

If `vehicle_local_position` has no `_v1`, set `topics_names.local_position_topic`
in `config/exp/t2_hw_param.yaml`. Same for `status_topic`. No rebuild needed.

**The camera calibration is keyed on the RESOLUTION.** camera_ros builds the
calibration name as `<model>_<camera id>_<WIDTHxHEIGHT>` (CameraNode.cpp, "format
camera name for calibration file"), so the file in `~/.ros/camera_info/` is
resolution-specific and camera_info_manager does not rescale. Nothing needs doing
to it when you clone a new workspace -- it lives outside the workspace -- **as
long as the resolution stays 1920x1200**, which the launch file now pins. Change
the resolution and you must recalibrate.

**Camera calibration is a hard prerequisite.** The detector publishes the
`platform` transform only when `calibrated` is true, and the EV bridge is built
on that transform. No calibration means no platform TF, no external vision, and
no indoor position estimate — while corner detections keep publishing, so the
detector *looks* healthy.

```bash
ros2 topic echo /tf --once | grep -A3 platform      # must show `platform`
# and watch the detector for: "The camera is not calibrated!"
```

Reproduced deliberately on a machine without a calibration, so you know what it
looks like -- note that the failure cascades and only the FIRST line names the
real cause:

```
[camera_calibration_parsers] Unable to open camera calibration file [...1920x1080.yaml]
[camera.camera] Camera calibration file ... not found
[camera.rectify] Rectified topic '/camera/image_rect' requested but camera
                 publishing '/camera/camera/camera_info' is uncalibrated
```

No rectified image means no detections, no `platform` TF, no EV, no position.

**K vs P.** The detector gets the rectified image but the unrectified
`camera_info`, and both it and this stack read `K`. Compare `k[0],k[4],k[2],k[5]`
against `p[0],p[5],p[2],p[6]` in `/camera/camera/camera_info`. If they differ
materially, that is a percent-level scale error on every range.

**Check the camera feed before anything else.** Start it on its own:

```bash
ros2 launch apriltag_ros camera_36h11.launch.yml
```

then, in another terminal:

```bash
ros2 topic hz /camera/camera/image_raw          # expect ~15 Hz
ros2 topic echo /camera/camera/camera_info --once
ros2 topic echo /apriltag/detections            # tags, with corner pixels
ros2 topic echo /tf --once | grep -A3 platform  # the transform the EV bridge needs
```

For an actual picture, headless over SSH:

```bash
ros2 run image_view image_saver --ros-args \
  -r image:=/camera/image_rect -p filename_format:=/tmp/frame%04i.jpg
```

then look at `/tmp/frame0000.jpg`. With a display or X forwarding,
`ros2 run rqt_image_view rqt_image_view` is easier.

**Watch the camera_ros startup log for a resolution adjustment.** If the sensor
cannot give exactly what the launch asks for, camera_ros silently picks the
nearest and says so once:

```
stream configuration adjusted from "1920x1200-..." to "1920x1080-..."
```

That matters more than it looks: the calibration filename contains the
resolution, so an adjusted stream looks for a calibration file that does not
exist, and you get the uncalibrated chain below.

**Link rate.**

```bash
ros2 topic hz /fmu/out/vehicle_odometry     # the controller's input, wants >=100 Hz
ros2 topic hz /fmu/in/actuator_motors       # confirms we publish at 100 Hz
```

## 3. PX4 parameters

### Outdoors, with GNSS

This is the configuration every result in CLAUDE.md was measured with, so it is
the better-validated one. Restore these four:

| parameter | indoor value | **outdoor** | why |
|---|---|---|---|
| `EKF2_GPS_CTRL` | 0 | **7** | GNSS fusion back on |
| `EKF2_EV_CTRL` | 1 | **0** | the tag is not the position source any more |
| `EKF2_HGT_REF` | 2 (range) | **1** (GPS) | what the SITL work used; range stays an AID |
| `NAV_RCL_ACT` | 3 (Land) | **2** (Return) | RTL is the right RC-loss action outdoors |

And launch with `ev_bridge:=false`, so the bridge is not publishing into a topic
nothing fuses.

**Keep these at their new values -- they are not indoor-specific:**

| parameter | value | why it stays |
|---|---|---|
| `EKF2_RNG_CTRL` | **2** | conditional aiding drops out above `EKF2_RNG_A_VMAX` = 1.0 m/s, which is exactly Phase 1's `max_velocity_xy_` -- it flickered through every approach |
| `EKF2_RNG_POS_X/Y/Z` | measured | a physical lever arm; still unmeasured on this vehicle |
| `COM_DISARM_LAND` | **0** | the land detector is meaningless in offboard direct-actuator mode, GNSS or not |
| `EKF2_OF_CTRL` | **0** | no flow sensor is fitted either way |

`EKF2_EVP_NOISE` and `EKF2_EV_DELAY` stop mattering once `EKF2_EV_CTRL` is 0;
leave them wherever they are.

### Indoors, no GNSS

```
EKF2_EV_CTRL 1        bit 0 only: horizontal position
EKF2_GPS_CTRL 0       no GNSS indoors
EKF2_HGT_REF 2        rangefinder as height reference, baro as aid
EKF2_RNG_CTRL 2       always on (currently 1 = conditional on the vehicle)
EKF2_RNG_POS_X/Y/Z    the TFmini's lever arm (currently all 0.0 — measure it)
EKF2_EV_DELAY         measured camera+detector latency, in ms
EKF2_EV_POS_X/Y/Z 0   the bridge already applies the camera lever arm
EKF2_EV_NOISE_MD 0    use the variance the bridge publishes
EKF2_OF_CTRL 0        no flow sensor fitted
COM_DISARM_LAND 0     the land detector is meaningless in direct-actuator mode
```

Leave the RC failsafe and the kill switch alone.

## 4. The extrinsic check — do this first, on the ground

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600        # adjust to your wiring
ros2 launch px4_offboard_lowlevel t2_hw.launch.py camera_only:=true
ros2 topic echo /landing/tag_in_body
```

Nothing that can command the aircraft is started. Move the **aircraft**, not the
tag:

| aircraft position | expected `/landing/tag_in_body` |
|---|---|
| directly above the tag | `[0, 0, -h]` |
| moved 0.5 m **forward** | `x -> -0.5` |
| moved 0.5 m **left** | `y -> -0.5` |

Both lateral signs wrong is a 180 deg boresight error; one axis wrong is 90 deg.
Fix with `camera_rotation_z_deg` / `camera_rotation_x_deg` in the yaml.

## 5. Bench, props off

```bash
ros2 launch px4_offboard_lowlevel t2_hw.launch.py controller_type:=stsmc
ros2 topic echo /landing/wrench          # published before the offboard gate
```

Tilt the airframe by hand and check `tau_x`/`tau_y` respond in the right sense.
Then arm into offboard, still props off, and verify the tilt servos move — they
are on **Servo 4 and 5** on this vehicle (`CA_SV_CS_COUNT` is 3), not 5 and 6 as
in SITL. Command -1 / 0 / +1 and measure the physical angle with an inclinometer;
PX4's allocator is bypassed in direct-actuator mode, so `CA_SV_TL*_MINA/MAXA` are
**not** applied to what we publish.

## 6. Flying, in order

Take off **manually on the RC in Altitude mode** every time. The camera sits
~5 mm above the tag on the gear, so nothing decodes on the pad and there is no
horizontal aiding during the climb. Confirm `xy_valid` before handing over.

```bash
# 1  PX4's own controller holds station on the tag
ros2 launch px4_offboard_lowlevel t2_hw.launch.py controller_type:=px4

# 2  same, with steps
ros2 launch px4_offboard_lowlevel t2_hw.launch.py controller_type:=px4 enable_steps:=true

# 3  the STSMC
ros2 launch px4_offboard_lowlevel t2_hw.launch.py controller_type:=stsmc
ros2 launch px4_offboard_lowlevel t2_hw.launch.py controller_type:=stsmc enable_steps:=true
```

Step 1 is the one not to skip: it uses none of this project's control code, so if
it cannot hold, the estimate is the problem and nothing measured later means
anything.

**Losing the tag loses your entire horizontal estimate.** EKF2 falls straight
back to fake-position fusion, mid-flight, with no warning on any topic the
controller reads — measured in SITL as a 49 m departure while the estimate
reported a perfect hold. Keep a finger on the mode switch and rehearse "tag out
of frame" as an abort.

## Pre-flight checklist

Grouped by risk, not by convenience. Everything in A is done with props OFF.

### A. Bench, powered, props off

- [ ] `ros2 topic list | grep fmu/out` -- confirm the `_v<N>` suffixes match
      `topics_names.*` in the yaml (`vehicle_local_position_v1` confirmed; check
      `vehicle_status` too).
- [ ] `ros2 pkg prefix apriltag_ros` points at the workspace, not `/opt/ros/jazzy`.
- [ ] `ros2 topic hz /fmu/out/vehicle_odometry` -- wants >=100 Hz. This is the
      controller's input; below that the loop is starved.
- [ ] `camera_info`: width/height really 1920x1200 (no "stream configuration
      adjusted" line), and `k[]` vs `p[]` compared.
- [ ] **Tag size measured with a ruler** -- the black square's outer edge, not the
      printed sheet. PnP range scales linearly with it.
- [ ] `/landing/tag_in_body` signs, with `camera_only:=true`. This is the only
      end-to-end check of `R_b_cam`; the detector-level TF check validates the
      camera but not the body rotation.
- [ ] **`camera_offset_body.z`**, now checkable since the scale is right: park the
      aircraft at a known height over the tag and compare the tag-derived range
      against `/fmu/out/distance_sensor`. They should agree once the lidar/camera
      lever arm is accounted for. A residual difference IS the offset error.
- [ ] **Tilt servos**: arm into offboard, props off, and confirm BOTH tilts move.
      They are on Servo 4 and 5 (`tilt_1_servo_index: 3`, `tilt_2_servo_index: 4`)
      because `CA_SV_CS_COUNT` is 3 on this aircraft. If one sits still while the
      other tracks, the indices are wrong -- and differential tilt is the yaw axis.
- [ ] **Tilt travel**: command -1 / 0 / +1 and measure the physical angle with an
      inclinometer. `CA_SV_TL*_MINA/MAXA` are NOT applied in direct-actuator mode,
      so `tilt_min_deg`/`tilt_max_deg` must match what the servo physically does.
- [ ] `/landing/wrench` responds in the right sense when you tilt the airframe by
      hand. It publishes before the offboard gate, so nothing needs to be armed.
- [ ] PX4 parameter block above applied.

### B. Manual flight, RC only, no offboard

- [ ] Hover in Altitude mode and **keep the ULog**. Read the steady
      `actuator_motors` value: this config predicts **~0.65**, and it is the
      single most likely thing to be wrong, because `omega_to_pwm_coefficient` is
      still the x500's. A large discrepancy means the aircraft will jump or sag at
      handover. Correct `x_0` so the curve passes through the measured point.
- [ ] While hovering over the tag, confirm the estimator has actually taken the
      tag: `cs_ev_pos: True` in `estimator_status_flags`, `xy_valid: True`, and
      sane innovations in `estimator_aid_src_ev_pos`.
- [ ] **Position mode, then let go of the sticks.** No offboard, none of this
      project's control code. If it cannot hold here, the estimate is the problem
      and nothing measured afterwards means anything.

### C. Offboard

- [ ] `controller_type:=px4` hold, then `enable_steps:=true`.
- [ ] `controller_type:=stsmc` hold, then steps.

Losing the tag loses the whole horizontal estimate, mid-flight, with no warning on
any topic the controller reads. Rehearse it as an abort: hand back to Altitude
mode on the RC.

## Still open

- Hover once and compare the steady `actuator_motors` value against the ~0.65
  this config predicts. `omega_to_pwm_coefficient` is still the x500's.
- Weigh the aircraft; CAD says 2363 g, the config now says 2387 g.
- `EKF2_RNG_POS_*` are 0.0 on the vehicle and the TFmini's lever arm is unmeasured.

# T2 indoor bring-up (no GNSS)

Clone, build, launch. Everything below has been exercised in SITL; **nothing in
it has flown on the real aircraft**, so treat every step as a test rather than a
procedure.

## 1. Workspace

```bash
mkdir -p ~/t2_ws/src && cd ~/t2_ws/src
git clone -b feature/indoor-ev-hover git@github.com:ppgerard/px4-offboard-smc.git
git clone git@github.com:ppgerard/apriltag_ros_enhanced.git
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs && git checkout 56f8019425a16af2941df5a9288834398681e394 && cd ../..

source /opt/ros/jazzy/setup.bash
source ~/<YOUR OLD WORKSPACE>/install/setup.bash    # camera_ros + libcamera, see below
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Verified from scratch: the three repos build clean in ~4 min on a desktop, so
allow 10-15 on the Pi. `t2_indoor.repos` describes the same set if you prefer
`vcs import src < src/px4-offboard-smc/t2_indoor.repos`, but plain clones are
listed first because `vcs` is not installed everywhere.

**You must overlay your existing workspace**, and the order above matters.
`camera_ros` and `libcamera` are deliberately NOT in the new workspace, so the
launch can only find them through the old one. Do not be tempted to
`apt install ros-jazzy-camera-ros ros-jazzy-libcamera` instead: the apt libcamera
is upstream, while a Raspberry Pi camera needs the Pi's own fork -- which is
exactly what you built (`libcamera v0.5.2+rpt20250903`, `camera_ros` rel_05).
Those two are consistent with each other and with a working camera; leave them
alone.

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
ros2 launch px4_offboard_lowlevel t2_hw_indoor.launch.py camera_only:=true
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
ros2 launch px4_offboard_lowlevel t2_hw_indoor.launch.py controller_type:=stsmc
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
ros2 launch px4_offboard_lowlevel t2_hw_indoor.launch.py controller_type:=px4

# 2  same, with steps
ros2 launch px4_offboard_lowlevel t2_hw_indoor.launch.py controller_type:=px4 enable_steps:=true

# 3  the STSMC
ros2 launch px4_offboard_lowlevel t2_hw_indoor.launch.py controller_type:=stsmc
ros2 launch px4_offboard_lowlevel t2_hw_indoor.launch.py controller_type:=stsmc enable_steps:=true
```

Step 1 is the one not to skip: it uses none of this project's control code, so if
it cannot hold, the estimate is the problem and nothing measured later means
anything.

**Losing the tag loses your entire horizontal estimate.** EKF2 falls straight
back to fake-position fusion, mid-flight, with no warning on any topic the
controller reads — measured in SITL as a 49 m departure while the estimate
reported a perfect hold. Keep a finger on the mode switch and rehearse "tag out
of frame" as an abort.

## Still open

- Hover once and compare the steady `actuator_motors` value against the ~0.65
  this config predicts. `omega_to_pwm_coefficient` is still the x500's.
- Weigh the aircraft; CAD says 2363 g, the config now says 2387 g.
- `EKF2_RNG_POS_*` are 0.0 on the vehicle and the TFmini's lever arm is unmeasured.

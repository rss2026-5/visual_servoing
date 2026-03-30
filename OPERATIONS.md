# Lab 4 Operations Guide

## Prerequisites
- Docker running on the car
- SSH access to the car (`ssh racecar@192.168.1.<CAR_NUMBER>`)
- Teleop running in a separate terminal on the car

---

## 1. Deploy Code to the Car

From your laptop, run:
```bash
./scripts/deploy.sh          # defaults to car 102
./scripts/deploy.sh 104      # specify a different car number
```

Then SSH into the car, connect to Docker, and build:
```bash
cd ~/racecar_ws && colcon build --symlink-install && source install/setup.bash
```

---

## 2. Cone Parking Mode

Drives autonomously to park 0.3m in front of an orange cone.
Hold **RB** on the joystick to enable autonomous driving.

```bash
ros2 launch visual_servoing parking_deploy.launch.xml
```

## 3. Line Follower Mode

Follows an orange tape line on the ground. Pass `line_follower:=true` — no file edits needed.

```bash
ros2 launch visual_servoing parking_deploy.launch.xml line_follower:=true
```

Hold **RB** on the joystick to enable. The car drives at 1.0 m/s and will not reverse.

Both modes launch:
- `safety_controller` — stops the car if an obstacle is too close
- `cone_detector` — detects the cone or line from the camera feed
- `homography_transformer` — converts pixel location to real-world coordinates
- `parking_controller` — drives the car

To visualize detection in VNC (`rqt` → Plugins → Visualization → Image View):
```
Topic: /cone_debug_img_compressed
```

---

## 4. YOLO Annotator

Runs YOLOv11 object detection on the live ZED camera feed.
Requires the ZED camera node to already be running.

```bash
ros2 launch visual_servoing yolo_annotator.launch.xml
```

To view annotated output in VNC (`rqt` → Plugins → Visualization → Image View):
```
Topic: /yolo/annotated_image
```

---

## 5. Simulation (No Robot Required)

Runs the parking controller against a simulated cone using an interactive marker in RViz.

```bash
ros2 launch visual_servoing parking_sim.launch.xml
```

---

## 6. Monitor Error Topics

In VNC, open `rqt` → Plugins → Visualization → Plot, then add:
```
/parking_error/x_error
/parking_error/y_error
/parking_error/distance_error
```

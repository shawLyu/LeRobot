# Camera capture, preview and perception

## Live check: 2026-10-11, Beijing time

The host `lerobot` at `/home/xy/Research/lerobot` currently enumerates **one**
USB Camera, exposing `/dev/video0` and `/dev/video1`. The configured capture
path ending in `video-index0` resolves to `/dev/video0`. These two nodes belong
to one camera. The previously used icspring camera is absent from the current
device inventory; its commands below apply after it is reconnected.

The existing remote `scripts/camera_x11.py` successfully captured a 640x480
snapshot, with the backend reporting 30 fps, saved at
`logs/camera-check-20261011.png` on the remote host. The reported fps is the
backend setting, not a measured sustained stream rate. PipeWire had an open
camera descriptor; the snapshot still succeeded without stopping it. This check
did not test the X11 live window or the new tabletop worker.

The remote host still lacks `src/lerobot_arm_control/runtime/tabletop_server.py`.
Raw preview and snapshots work through the existing script. The unified
`view tabletop` command requires deploying the current source first. Local
documentation was updated during the October 11 source cleanup; remote README
and camera documentation differ from the current local copies.

## Choose an entry

| Need | Entry | Configuration |
| --- | --- | --- |
| Raw live image on Mac | Local `scripts/view_camera_x11.sh` | Remote `configs/camera.toml`, optional preview overrides |
| One image without X11 | Remote `.venv/bin/python scripts/camera_x11.py --snapshot ...` | Remote `configs/camera.toml` |
| Tracked objects, markers and recorded observations | Local `scripts/lerobot-arm view tabletop --camera-only` after deployment | Remote `camera.toml` selects the device; `tabletop.json` supplies perception and calibration |
| Frames in a Python program | Remote OpenCV `VideoCapture`, example below | Remote `configs/camera.toml` |

Default capture uses USB Camera's `video-index0` and its stable USB path,
independent of video numbering after reconnection.
`configs/camera.toml` requests 640x480 and targets 15 display fps by default.

The remote runtime uses OpenCV headless, Pillow, and system Tk 8.6.
OpenCV captures frames; Tk displays them over X11, so the existing OpenCV package need not be replaced.
Only the latest frame is retained to avoid accumulating stale images on slower SSH links.

The existing Mac uses XQuartz 2.8.6. Run from the local project root:

```bash
./scripts/view_camera_x11.sh
```

After reconnecting the second camera, confirm that its path exists with
`ssh lerobot 'ls -l /dev/v4l/by-id/'`, then view it in another local terminal:

```bash
./scripts/view_camera_x11.sh --device /dev/v4l/by-id/usb-icSpring_icspring_camera-video-index0
```

The wrapper forwards `--device`, `--width`, `--height`, `--config`, `--snapshot`, and other preview arguments.
It always starts XQuartz and uses X11 forwarding, including with `--snapshot`.
Use plain SSH as shown below when saving a snapshot without X11.
The window title identifies the device. Two cameras can have separate windows;
close Push-T or other capture programs before opening an occupied camera.
Prefer by-id paths because video numbers can change.
`LEROBOT_HOST` and `LEROBOT_REMOTE_ROOT` override the wrapper's SSH destination and project directory.

## Resolution

```bash
./scripts/view_camera_x11.sh --device /dev/video0 --width 1920 --height 1080
./scripts/view_camera_x11.sh --device /dev/video2 --width 1280 --height 720
```

Earlier device enumeration reported USB Camera support up to 1920x1080 and
icspring up to 1280x720, with both supporting 640x480, 800x600 and 1280x720.
Startup reports requested and actual dimensions.
Persistent preview defaults are remote `configs/camera.toml` fields `width` and `height`.
CLI sizes affect only that launch; restart to change resolution.
Independent preview configuration does not modify Push-T intrinsics or capture dimensions.

## Remote desktop and manual forwarding

Run directly in the remote graphical desktop terminal:

```bash
cd /home/xy/Research/lerobot
.venv/bin/python scripts/camera_x11.py --device /dev/video0
# Another desktop terminal:
.venv/bin/python scripts/camera_x11.py --device /dev/video2
```

That terminal needs a valid DISPLAY. For ordinary SSH from Mac, use XQuartz and the local wrapper.
Equivalent manual forwarding command on the Mac:

```bash
open -a XQuartz
export DISPLAY="$(launchctl getenv DISPLAY)"
ssh -X -C -o XAuthLocation=/opt/X11/bin/xauth \
    -o ForwardX11Timeout=0 -o ControlMaster=no -o ControlPath=none lerobot \
    'cd /home/xy/Research/lerobot && ./scripts/uv run --locked --extra hardware scripts/camera_x11.py'
```

This uses untrusted `-X` forwarding, without `-Y` or opening X11 TCP ports.
After first XQuartz installation, older terminals may lack DISPLAY; the wrapper reads its launchctl address.
The status bar reports frames, actual display rate, and frame age.
Esc or closing the window releases the camera and ends SSH.
Actual rate depends on network/X11 transfer; lower resolution reduces bandwidth.
Preview does not record video, change exposure, or operate motors.

The existing remote account xy belongs to video. Group changes require a new SSH login.
For permission failures, check `id`, `ls -l /dev/video0`, and camera ownership by other applications.

## Snapshot without X11

Save one image on the remote host without a GUI:

Run on the Mac, using the already installed remote environment:

```bash
ssh lerobot 'cd /home/xy/Research/lerobot && .venv/bin/python scripts/camera_x11.py --snapshot logs/camera-snapshot.png'
scp lerobot:/home/xy/Research/lerobot/logs/camera-snapshot.png ./logs/camera-snapshot.png
```

The snapshot path is remote; `scp` copies it locally. Use a new filename to
retain earlier captures. A snapshot does not require `DISPLAY` or XQuartz.

## Tabletop perception after deployment

Run from the local checkout:

```bash
./scripts/deploy_code.sh --dry-run
./scripts/deploy_code.sh

# Camera and perception view; no motor bus is opened.
./scripts/lerobot-arm view tabletop --camera-only --fps 5

# Finite capture without a local window; choose a fresh output directory.
./scripts/lerobot-arm view tabletop --camera-only --headless \
  --duration 10 --fps 5 --output logs/camera-session-01
```

Deployment preserves existing remote device configuration and calibration.
For a different host, deployment uses `LEROBOT_HOST` and
`LEROBOT_REMOTE_ROOT`; the tabletop client uses `--host` and `--remote-root`.
This tabletop entry uses the local Python/MuJoCo environment and does not need
XQuartz. It saves `frames/`, `annotated/`, available native-resolution images
in `native/`, `states.jsonl`, `latest.json` and session metadata in
`calibration.json` locally. `--fps` controls the display stream rate.

The device comes from remote `camera.toml`; tabletop capture dimensions,
intrinsics, marker definitions and focus controls come from remote
`tabletop.json`. When configured, the worker applies and verifies fixed camera
focus, and can save observed table registration. `--camera-only` prevents motor
bus access; it still runs the camera/perception pipeline. The current calibration
belongs to the configured USB Camera. Switching to icspring requires calibration
for that camera before using measured coordinates.

## Frames from Python

Run this in the remote project with `.venv/bin/python`; the camera is attached
to Linux, so a local Mac `VideoCapture('/dev/video0')` cannot access it:

```python
import tomllib

import cv2

with open('configs/camera.toml', 'rb') as stream:
    settings = tomllib.load(stream)['camera']
camera = cv2.VideoCapture(settings['device'], cv2.CAP_V4L2)
try:
    if not camera.isOpened():
        raise RuntimeError('Camera could not be opened')
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, settings['width'])
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, settings['height'])
    for _ in range(5):
        ok, frame = camera.read()
        if not ok:
            raise RuntimeError('Camera returned no frame')
    # frame is a BGR NumPy array. Pass it to your processing code here.
    print(frame.shape)
finally:
    camera.release()
```

Close an active capture application before starting another reader on the same
camera. To inspect current users, run `ssh lerobot 'fuser /dev/video0'`.

## Earlier X11 verification

In the measured check, 640x480 capture reported 30 fps; forwarded DISPLAY was `localhost:10.0`.
The local `LeRobot USB Camera — live via ssh -X` window received and displayed frames continuously.
`logs/camera-snapshot.png` is a static verification frame; live video appears in XQuartz.

See [XQuartz installation](https://www.xquartz.org/releases/bare/XQuartz-2.8.6.html).

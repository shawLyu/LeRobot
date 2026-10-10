# Live camera preview over SSH -X

The existing remote setup has two cameras: USB Camera exposes video0/video1, and icspring exposes video2/video3.
Each pair belongs to one USB device. Default capture uses USB Camera's `video-index0` and its stable USB path,
independent of video numbering after reconnection.
`configs/camera.toml` requests 640x480 and targets 15 display fps by default.

The remote runtime uses OpenCV headless, Pillow, and system Tk 8.6.
OpenCV captures frames; Tk displays them over X11, so the existing OpenCV package need not be replaced.
Only the latest frame is retained to avoid accumulating stale images on slower SSH links.

The existing Mac uses XQuartz 2.8.6. Run from the local project root:

```bash
./scripts/view_camera_x11.sh
```

View the second camera in another local terminal:

```bash
./scripts/view_camera_x11.sh --device /dev/v4l/by-id/usb-icSpring_icspring_camera-video-index0
```

The wrapper forwards `--device`, `--width`, `--height`, `--config`, `--snapshot`, and other preview arguments.
The window title identifies the device. Two cameras can have separate windows;
close Push-T or other capture programs before opening an occupied camera.
Prefer by-id paths because video numbers can change.
`LEROBOT_HOST` and `LEROBOT_REMOTE_ROOT` override the wrapper's SSH destination and project directory.

## Resolution

```bash
./scripts/view_camera_x11.sh --device /dev/video0 --width 1920 --height 1080
./scripts/view_camera_x11.sh --device /dev/video2 --width 1280 --height 720
```

The current USB Camera supports up to 1920x1080; icspring up to 1280x720.
Both support 640x480, 800x600, and 1280x720. Startup reports requested and actual dimensions.
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

## Snapshot and measured check

Save one image on the remote host without a GUI:

```bash
cd /home/xy/Research/lerobot
./scripts/uv run --locked --extra hardware scripts/camera_x11.py --snapshot logs/camera-snapshot.png
```

In the measured check, 640x480 capture reported 30 fps; forwarded DISPLAY was `localhost:10.0`.
The local `LeRobot USB Camera — live via ssh -X` window received and displayed frames continuously.
`logs/camera-snapshot.png` is a static verification frame; live video appears in XQuartz.

See [XQuartz installation](https://www.xquartz.org/releases/bare/XQuartz-2.8.6.html).

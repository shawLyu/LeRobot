# Six-motor servo workbench

Start from the local project root:

```sh
./scripts/deploy_code.sh
./scripts/lerobot-arm view motors
```

Open **http://127.0.0.1:8765**. Choose `follower` (black arm) or `arm1` (white
leader), set the six channel IDs and bus baud rate, then connect. SSH defaults come
from `configs/runtime.json`; `--host`, `--remote-root` and `--port` override them.
The application needs only Python's standard library locally; no Node service,
browser extension, cloud service or frontend build is required.

## Read and configure

The six slots stay visible even when only one motor is installed. Offline motors
are excluded from feedback reads and motion writes. Connecting only opens and
inspects the selected bus; it does not enable torque, initialize an old goal or
reset the robot. Selecting another arm disconnects the previous worker first.
The serial ownership lock prevents another following/calibration process from
controlling the same port concurrently.

Click a motor to select its parameter table. The table contains **58 documented
fields**: 29 EEPROM settings, six RAM controls, identity, live feedback and factory
limits. Fields show address, raw units, permitted values, current value and new
value. Filter by name/address/storage, or export/import a parameter JSON file.
Importing prepares edits for review; it never immediately writes to a motor.

After releasing all online motors' torque, edit parameters and review the exact
motor/ID, original values and proposed values:

- **Temporary write:** storage stays locked; EEPROM values affect the current
  powered session. They revert to saved values on power cycle.
- **Save to EEPROM:** the transaction unlocks storage, writes changed settings,
  verifies each value and relocks. Partial ordinary configuration failures or
  interrupted transactions attempt restoration and retain the readback evidence.
- RAM settings are not EEPROM settings and cannot be saved using that checkbox.
- A changed original value rejects stale edits rather than silently overwriting it.

Read-only firmware/feedback/factory fields and reserved address 7 are displayed
without a write input. Lock 55 is managed by transactions; it cannot be left
unlocked through a normal edit. Torque and target-position fields use the same
enable/release and motion-envelope checks as their dedicated controls. Velocity
and time/PWM registers cannot be used as an alternate motion route in other modes.
The editor is an STS3215 position-test tool, not a firmware flasher or raw-byte editor.

ID/baud changes require only one online motor and a single changed field. A new
ID is checked against configured channels and probed on the bus. The worker adopts
the new ID/baud for independent readback. If the firmware needs a restart or does
not respond at the new settings, the transaction reports the communication error;
disconnect, power-cycle as specified by the manufacturer and reconnect using the
new values. The connection form exposes both settings. These edits do not silently
rewrite `configs/arms.toml`; update that machine configuration before using normal
demos with a permanently changed bus assignment.

Changing zero, phase, scale, mode or EEPROM position limits invalidates the current
motion envelope. Set and inspect the new envelope before enabling again.

## Move and measure

1. Inspect the installation and the selected motors' actual mechanical travel.
   The initial envelope is the current position ±114 ticks (about ±10°), bounded
   by EEPROM limits. EEPROM limits alone do not prove a collision-free range.
2. Confirm the inspected travel in the interface. Keep the initial envelope or
   explicitly set lower/upper raw counts. The current position must be inside it.
   Stop motion and release the current motor's torque before setting this range.
   The editor retains each motor's draft while feedback refreshes and while
   switching between motors. The status below it shows the applied range, any
   unapplied input and the reason setting is disabled. A rejected edit keeps its
   draft. This software range lasts for the current connection; it does not change
   EEPROM `Min_Position_Limit` / `Max_Position_Limit` in the parameter table.
3. Check the channels to control, then enable them. The worker first seeds each
   goal from fresh actual position. Position motion requires mode 0, encoder scale
   1, Goal_Time=0 and locked storage. No reset-to-zero command is issued.
4. Enter a raw position and a ramp speed to move the current motor. Expand
   **independent six-motor targets** to send different positions to checked motors.
   A normal move ramps goals on the robot host, without browser/network timing in
   the ramp loop. It drives the native position loop, without the task outer PID.
5. For response measurement, set full step angle, repetitions and hold duration.
   A 5° step means approximately −2.5° to +2.5° around the live starting position.
   Checked motors are tested sequentially, with both step directions and a return
   target for each motor. The worker refuses the entire plan before motion if any
   target is outside the configured envelope.

The worker samples at a requested 100 Hz and timestamps feedback and command
writes locally on the robot host. The browser reports actual sample cadence and
refreshes a decimated display independently. Position, error, raw speed, native
output, current, voltage and temperature are available in live curves. Response
analysis uses the full remote samples and separates command-to-90%, 10–90% rise,
settling, final 300 ms error and overshoot. Select each tested motor to inspect its
curves; the table retains both directions for every tested channel.

Each of the four charts has independent **A/B measurement cursors** below it.
Choose horizontal (`Δt`), vertical (`Δy`) or both axes; click A or B to select a
cursor, then click the plot or drag its line/intersection. Arrow keys move the
selected cursor by 0.1% of the visible axis span; Shift uses 1%. Coordinates stay
in data units when the window size or display pixel density changes. Differences
are signed **B − A**; time differences use milliseconds and vertical differences
use the chart's displayed units. Free cursors measure coordinates; **snap to sample**
selects the closest visible sample in screen distance and identifies its curve.
It does not interpolate values between samples.

Enabling a live-chart measurement pauses display updates to keep the measured
waveform still. Remote acquisition, heartbeat and motor control continue. **Resume
display** exits live measurements; clearing the display or reconnecting clears
the old cursors. Changing the displayed quantity, selected channels, response
motor or response result resets their coordinates. Live measurements use feedback
received by the browser; performance acceptance still uses the remote response
samples and metrics. PNG export retains the cursor lines drawn on the chart.

**Acquisition completed is not a performance pass.** The 50 ms target refers to
command-to-90%, not movement onset. Strict error below one quantized tick requires
every settled sample to equal the target; an average near zero is insufficient.
Internal encoder agreement does not establish external absolute shaft accuracy.

Export live or response CSV, complete JSON, parameter JSON and curve PNGs from the
interface. Live export retains the most recent 12,000 remote frames; the browser's
live plot shows the most recent 20 seconds. Export full recordings after motion
finishes. Response and parameter transactions also save under the remote directory
`logs/servo-workbench/<session>/`. If disk saving fails, feedback/holding still run,
the result reports `log_error`, and in-memory data can be exported.

## Stop, disconnect and recover

**Stop and hold** cancels the ramp/response and commands fresh healthy feedback
positions. It does not resend goals to a motor with a latched device fault.
**Release torque** is separate: use it deliberately when the mechanism is supported.
Disconnecting/closing the worker requests position hold for its enabled motors;
it does not automatically drop a loaded arm by releasing all torque.

No browser heartbeat for 1.5 seconds cancels motion, holds healthy channels and
requires a new enable action. Stale UI feedback disables new motion controls while
stop/release remain available for an existing session. The worker stops new motion
on lost torque, feedback outside the envelope, non-voltage device alarms, the
configured temperature limit, excessive step overshoot or loaded stall. Voltage
bit 0 remains a visible warning, following the user's chosen policy. Transport
failure requests a hold and ends the worker; no automatic motion restart occurs.
After inspection, clear the fault and enable explicitly. Fault recovery may relock
an unloaded device, but does not reset gains, clear hardware protection by sending
a new goal, or resume an interrupted trajectory.

The local HTTP service binds only to loopback, validates Host/Origin and requires
a per-session token for APIs. It is not an unauthenticated network motor server.

## Implementation and verification

```text
workbench/
  registers.py    canonical widths, access, units and value constraints
  hardware.py     exclusive discovery and raw alarm-aware SDK access
  controller.py   six-channel transactions, motion, response sampling and guards
  worker.py       remote clock and bounded JSON-lines command loop
  link.py         local SSH connection, state cache and command acknowledgements
  server.py       loopback HTTP API and CSV export
  web/           offline HTML, CSS and JavaScript/canvas interface
```

The SDK's overlapping startup-force width is corrected per bus: register 24 and
integral register 25 are each one byte. The shared installed SDK table is unchanged.

```sh
PYTHONPATH=src python -m pytest -q tests/test_servo_workbench.py tests/test_servo_workbench_http.py
```

The optional `tests/browser/servo_workbench.cjs` uses Playwright. Install Playwright
in a development environment that Node can resolve (or set `NODE_PATH`). With a
workbench HTTP service running, the default invocation checks the browser UI with
the state endpoint mocked offline and does not operate hardware:

```sh
node tests/browser/servo_workbench.cjs
node tests/browser/servo_workbench_bounds.cjs
node tests/browser/servo_workbench_measurement.cjs
```

The range-editor regression mocks every API and checks editing across feedback
refreshes, focus changes, motor selection, successful/rejected saves and disabled
controls. It never sends commands to hardware, including an already open session.

`--hardware` is a separate, physically gated acceptance for this repository's
recorded black ID 1 and simple-load installation. It verifies the applied-parameter
fingerprint, tests temporary/restored P, EEPROM save/readback, a 10-tick move,
heartbeat timeout and a 5° step, then releases torque and checks retained settings.
It must only be run with that prepared motor and inspected ±90° travel. It is not
an automatic test for arbitrary assembled arms. Set
`PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH` to a compatible installed Chrome if needed;
Playwright always creates a separate browser profile.

The functional acceptance record and physical response samples are linked in
[the workbench hardware report](measurements/servo-workbench-2026-10-16/README.md).
Six-channel sequencing and faults are also exercised with simulated bus fixtures;
only ID 1 is physically motion-tested in that record.

The feature reference is [Feetech FD](https://ftservo.github.io/ftservo-wiki/tools/fd/),
which provides search, configuration and basic motor testing. Seeed also lists a
[Feetech documentation hub](https://wiki.seeedstudio.com/feetech_servo/).
Register definitions follow the
[manufacturer STS table](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/),
not reserved bytes or field widths inferred from screenshots.

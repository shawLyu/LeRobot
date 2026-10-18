# Camera-guided T pick and place

The geometric policy picks the wooden T by its stem and places it on the black T
outline. Each run estimates the source and target from a fresh camera observation;
positions are not hard-coded. Source, target, orientation, joint limits, and swept
geometry must admit a complete plan. An arbitrary placement outside this feasible
workspace is rejected.

```sh
# Observe, plan, and run contact physics locally; no motion targets
./scripts/lerobot-arm task pick-t

# Permit physical execution after all preflight checks pass
./scripts/lerobot-arm task pick-t --execute

# Replay an existing observation without connecting to hardware
./scripts/lerobot-arm task pick-t --snapshot logs/my-run/snapshot \
  --output logs/my-offline-run
```

Use the installation, device setup, and calibration instructions in the
[README](../README.md). Only run one hardware client per arm. The live entry opens
the configured camera and reads follower encoders, including in preview mode.
`--execute` starts the finite sequence automatically once planning and simulation
pass. An offline snapshot cannot be combined with `--execute`.

## Calibration and observations

The camera intrinsics, primary table anchor, measured wood thickness, joint
calibration, and a saved follower base in the current table frame are required.
The table anchor and black target must remain visible during execution. The
source T must be fully observed before descent. Brief source occlusion during
descent/closing is allowed for at most two seconds. While carrying, the original
source observation supplies the grasp reference; it is not treated as a new
measurement of the airborne T.

Expose the primary table anchor and base ID 7 together once to register a stationary
base. The registration service saves a pose only after the mount fit and temporal
stability checks pass. Later runs read that pose without requiring ID 7 in view.
Moving the base, replacing marker geometry, or changing the table reference or
camera calibration requires a new accepted record. Rejected mount fits report
`pose_rejected`, their reprojection error, and a reason; seeing a decoded ID alone
does not establish a valid base pose.

On 2026-10-09, the exposed ID 7 was decoded, but the reconfirmed 42 mm black border
at 70 mm height produced about 7.1 native-pixel mount-fit error, exceeding the
4.5 native-pixel acceptance limit. No new fixed pose was accepted. Physical grasp
and placement have therefore not yet been validated with this camera setup.
The later [October 11 current scene](current-scene.md) corrected the black-border
corner selection and actual tabletop reference. Both bases now have accepted fixed
records; that update verifies observation and registration, not physical grasping.

## Planning and execution

All inverse kinematics, swept collision checks, and MuJoCo contact simulation run
locally. The payload remains a free body and uses the original modeled jaws;
simulation does not attach or weld it to the robot. A plan must lift and carry the
T at least 20 mm above the table and finish within 5 mm center error and 8 degrees
of target yaw. Two offline cases passed; another turn caused a simulated slip and
was rejected. Passing simulated contact is a prerequisite, not a measurement of
physical grasp reliability or Cartesian accuracy.

The sequence is `prepare`, `approach`, `descend`, `close`, `lift`, `transfer`,
`lower`, `open`, and `retreat`. Jaw stages hold the first five joints; arm stages
keep the jaw target constant. Contact and carrying moves are capped at 0.3 rad/s;
approach uses the shared trajectory setting. The policy uses the same
[`configs/robot.json`](../configs/robot.json) as manual control, Reset,
circles, and Push-T. The remote host receives bounded joint segments and checks
stage order, fresh feedback, model/config hashes, reference consistency, speed,
joint bounds, and the digest of the locally checked segments. It does not install
or run MuJoCo.

An encoder closure residual is required before lifting to reject an empty grasp.
This contact proxy is not a force sensor and cannot certify that the T is secure.
After release and retreat, success requires three distinct fresh, fully visible
camera observations within the 5 mm / 8 degree placement limits. Cached source
poses and encoder arrival cannot count as placement success.

Additional localized arms appear in local simulation at explicitly labeled preview
joint poses. These are unmeasured obstacles, so the simulation does not establish
collision clearance against an additional arm whose actual posture differs from
its preview. Inspect the physical swept workspace before hardware operation.

Camera/feedback loss, changed landmarks, a motor fault, rejected physics, or
tracking failure stops progression and holds the arm. `Ctrl+C` also requests hold;
supporting torque remains enabled. Restore clear observations and rerun to obtain
a new plan; the policy does not blindly resume a failed grasp.

## Output and deployment

Each run writes to a new `logs/pick-t-<timestamp>/` directory:

| File | Contents |
| --- | --- |
| `capture/` | Live camera/encoder observations and runtime metadata |
| `snapshot/` | Exact observation, model metadata, and joint calibration used for planning |
| `plan.json` | Nine stages, bounded segments, and collision sample counts |
| `simulation.json` | Actual simulated payload poses and acceptance result |
| `result.json` | Preview/physical status, completed stages, errors, and final verification |

Failed preflight may produce only capture and result files. Generated sessions are
ignored by Git. The wire telemetry field remains `push_t` for transport
compatibility and includes `task: pick_t`; commands use `pick_execute`,
`pick_stop`, and `pick_result`.

```sh
./scripts/deploy_code.sh
```

The runtime manifest sends the shared stage checks and remote executor, while
`pick_t.py`, `pick_t_client.py`, and MuJoCo remain local. Accepted calibration data
is managed separately from source deployment; see
[base registration](../README.md#63-register-the-robot-base-to-the-tabletop).

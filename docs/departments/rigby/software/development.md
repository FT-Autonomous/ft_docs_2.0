# Development

Work on the maintained software in [FT-Rigby](https://github.com/FT-Autonomous/FT-Rigby), not a copied sketch or an older Hardware code folder. The checked published revision for this documentation is `a54a513` (26 September 2026).

The deployed host still uses Python 3.8. Newer development machines are fine, but keep the runtime compatible and test any fallback dependencies. TOML loading uses `tomli` on older Python versions.

## Finding the Code

| Module | Main responsibility |
| --- | --- |
| `configuration.py` | Strict TOML parsing and cross-file checks |
| `controller_identity.py`, `device_inventory.py` | Stable board identity and discovery |
| `device_gate.py` | Role verification, enrolment and runtime selection |
| `firmware_sync.py` | Managed controller builds and updates |
| `ddt_protocol.py`, `ddt_mimic.py` | CAN encoding, vehicle state and runtime |
| `rigby_cmd_bridge.py` | ROS Ackermann input |
| `rigby_interface.py` | Serial commands and telemetry |
| `assi.py` | ASSI reports and freshness |
| `remote_hid.py`, `remote_policy.py`, `remote_runtime.py` | Gamepad input, arming policy and remote route |
| `steering_drive_sequence.py` in `tools/` | Separate supervised diagnostic |

The [repository development guide](https://github.com/FT-Autonomous/FT-Rigby/blob/main/docs/DEVELOPMENT.md) is the fuller module and maintenance reference.

## Running the Software-Only Tests

From the FT-Rigby root on Linux:

```bash
export PYTHONPATH="$PWD/src/rigby_bridge"
python3 -m unittest discover -s tests -v
```

Or in PowerShell, using an installed Python environment with the project dependencies:

```powershell
$env:PYTHONPATH = "src/rigby_bridge"
python -m unittest discover -s tests -v
```

Report skips as well as passes. Linux route simulation uses pseudo-terminals and additional host dependencies; it is not equivalent to the Windows unit suite or to a real vehicle test.

The latest recorded local run had 272 tests and two skips. The [audit findings](../testing.md#controls-audit-open-findings) show why a passing suite can still miss an important fault. Add the failing regression before claiming one of those findings is closed.

## Change and Deployment Boundaries

Keep protocol fields, units and steering conventions explicit. An external angle in radians is not the controller's angle in degrees, and DDT's positive-left sign is not Rigby's actuator sign.

Changing a host module, changing a configuration file, compiling a controller and flashing it are different steps. Record which steps actually happened. Firmware source edits can trigger automatic updates on a later service start, even when the edit looks cosmetic.

Keep credentials, live machine configuration, unique controller identities and raw deployment backups out of the public examples. `local-artifacts/` is ignored; it is useful for outputs and recovery material, but is not backed up by a push. Share maintained code and concise test results through the normal repository paths.

Do not bundle a change to watchdogs, speed limits, firmware or calibration into an unrelated cleanup. Those deserve their own review and physical verification plan.

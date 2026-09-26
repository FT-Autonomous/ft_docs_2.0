# Remote Control

Remote control is being developed so we can move Rigby deliberately while checking sensors or collecting data, without needing the autonomous stack to choose a path. It is a separate debugging mission, not a substitute for the normal safety checks.

**Remote remains disabled in the recorded installation.** Input parsing and pairing have been exercised, but reliable wireless operation and powered vehicle acceptance are unfinished.

## Supported Controllers

The maintained reader supports Google Stadia controllers and standard Sony DualSense controllers over their supported USB/Bluetooth input paths. Use one supported pad at a time; ambiguous multiple-controller input is rejected.

A Stadia controller needs its Bluetooth-mode firmware before wireless use. Do not assume an old Wi-Fi-mode pad is ready just because the software recognises the product family. Controller conversion and pairing are separate from updating Rigby's motor firmware.

| Action | Stadia | DualSense |
| --- | --- | --- |
| Steering | Left stick | Left stick |
| Forward speed | Right stick up | Right stick up |
| Arm with neutral sticks and hold-to-run released | A | Cross |
| Hold to run | R1 | R1 |
| Stop | B | Circle |

Releasing R1 stops the request. After stopping or reconnecting, the controller must be armed again. Selecting ASSI mission 8 (`J`) chooses the route; it does not by itself authorise motion.

The extra remote cap is **0.05 m/s**, or the main cap if lower. The example uses a 12% deadzone and a **250 ms fresh-input deadline**. Holding a stick still must not turn a cached report into a new observation.

## Input-Only Check

Keep actuator power isolated. On the host, the following reads the pad without opening the motor-controller serial ports:

```bash
sudo -u rigby /opt/ft-rigby/venv/bin/python -m rigby_bridge.remote_hid --seconds 20
```

Exercise each stick, arm, stop and R1, then hold a steady input. Check both the report count and maximum gap. A successful Bluetooth connection icon is not evidence of fresh input.

The [repository remote-control guide](https://github.com/FT-Autonomous/FT-Rigby/blob/main/docs/REMOTE_CONTROL.md) contains pairing and input-check details. Pair only the identified controller, not every nearby device.

## Host-Specific Work

The ROCK's Bluetooth loader needed a board-name/firmware correction, and its older kernel required BlueZ's userspace HID route. The relevant configuration is retained in FT-Rigby, including the service override and `UserspaceHID=true` setting.

Those changes were made for the identified ROCK4C+ board revision. They are not a generic recipe to overwrite Bluetooth configuration on another host. Changing pads does not require reflashing the actuator controllers.

## What Still Fails

DualSense checks included healthy report windows, but also a **1.518-second gap**, followed later by an input/output error and a period with no reports despite BlueZ reporting a connection (22 September 2026). A later healthy window did not cancel the failed one, and no reliable range has been established.

Before enabling remote, resolve the freshness problem and the [shared control faults](../testing.md), then verify disconnect/reconnect, stop, re-arm, physical direction mapping and independent power removal in a supervised test. Do not lengthen the input watchdog to make intermittent Bluetooth look healthy.

The gamepad stop button and hold-to-run control are software inputs. They are not a certified remote emergency stop or a replacement for a physical cut-off.

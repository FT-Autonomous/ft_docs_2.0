# Firmware and Controllers

The drive and steering controllers each close their own feedback loop. The Linux host sends targets and checks telemetry; it is not directly generating the motor PWM.

## Current Boards

| Role | Board and source | Runtime connection |
| --- | --- | --- |
| Drive | FireBeetle ESP32; `ino/drive_motor_controller/` | USB serial, 115200 baud |
| Steering | UNO R4 WiFi; `ino/steering_controller/` | USB serial, 115200 baud |
| ASSI | Nano 33 IoT; `ino/assi_controller/` | USB serial, 9600 baud |

The drive board's CH340 USB bridge has no unique serial number. FT-Rigby uses the ESP32 factory chip identity with USB VID/PID guards instead. The UNO has a stable USB serial. Neither board should be selected from its current tty number.

The exact board profile matters. The managed drive target is `esp32:esp32:firebeetle32:UploadSpeed=115200`, not a generic ESP32 or D1 profile. Its pin aliases differ. Steering uses `arduino:renesas_uno:unor4wifi`. The [repository firmware guide](https://github.com/FT-Autonomous/FT-Rigby/blob/main/docs/FIRMWARE.md) retains the verified pin maps and toolchain details.

## Three Different Kinds of Limit

**Operational limits** in `rigby.toml` restrict what the host requests. **Compiled hard stops** in `controller-firmware.toml` restrict the controller firmware independently. **Sensor calibration** maps physical measurements to engineering units.

They are not substitutes for one another. In the last recorded configuration, the host steering limit was +/-7.5 degrees, the firmware command clamp was +/-8 degrees, and the potentiometer mapping covered +/-17.5 degrees. The recorded raw endpoints were 300 / 378 / 456, but these are not calibration values to copy onto a replacement mechanism without measurement.

The recorded drive command/overspeed caps were 0.15 m/s, with 300 ms communication watchdogs on both actuator controllers. Those depend on working feedback and the correct calibration; the [audit](../testing.md) identified gaps in sensor-failure handling.

A numeric steering target of zero requests centring and may energise the motor. **It is not the same as `STOP`.**

## Managed Updates

The firmware-sync service checks the configured source/settings against its saved fingerprints. A changed managed drive or steering role can be rebuilt and flashed at startup. ASSI is **not** part of that automatic flashing path.

Before updating a controller, isolate actuator power, confirm its exact identity and board profile, preserve the previous configuration, and agree the post-flash checks. A replacement board needs a supervised initial setup before normal identity-based updates can recognise it.

Even a comment change in a managed sketch can change its fingerprint and trigger a reflash. Keep firmware changes deliberate and separate from host-only documentation or formatting work.

The current fingerprints do not capture every possible toolchain/build-recipe difference. A state of `current` means the recorded synchronization checks matched; it is not a complete flash readback or proof that the controller is producing healthy telemetry.

## ASSI and Mission Selection

The host normally reads ASSI state rather than driving the indicator as an output device:

| Report | Meaning to the host |
| --- | --- |
| `O` | OFF; no driving authority |
| `R` | READY permission, not permission to move |
| `D` | DRIVING permission, alongside the selected route's other checks |
| `E` | Emergency/stop state |
| `J` | New remote mission selection, still subject to remote arming and checks |

The remote mission was added as mission 8 while preserving missions 1-7. Preserving the baseline also preserved a legacy state bug: after GO, the normal `D` report can immediately revert to `R`. The explicit manual sequence handles that narrowly for its diagnostic; it is not a repair of normal CAN/ROS operation.

The Nano 33 IoT uses **3.3 V logic**. Do not apply 5 V signals to it. Keep the [original electronics reference](../../hardware/electronics/electronics_overview.md) as history, but use the current firmware guide and actual wiring for the installed boards.

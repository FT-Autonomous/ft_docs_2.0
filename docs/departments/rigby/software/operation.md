# Setup and Operation

The normal installation lives at `/opt/ft-rigby` on the ROCK host, with a virtual environment and the `rigby` service account. The service entry point is **`rigby-device-gate.service`**. Do not start a second bridge beside it.

This is an orientation to the installed system, not an unattended deployment recipe. Use the repository's [installation and acceptance documentation](https://github.com/FT-Autonomous/FT-Rigby/blob/main/docs/INTEGRATION_ACCEPTANCE.md) for a new host. Keep motor power isolated during installation, controller identification and firmware work.

## What Happens at Startup

1. `rigby-firmware-sync.service` validates the two configuration files and checks the managed drive/steering firmware.
2. A changed managed role may be rebuilt and flashed to its identified board. Unchanged roles are not meant to be reflashed every boot.
3. The device gate identifies the USB boards and their actual protocols.
4. It starts the configured control route only after the required checks pass.

A reboot or service start is therefore **not necessarily a read-only diagnostic**. Firmware source or hard-stop configuration changes can cause an update before normal control starts.

The gate checks what a board reports, not just whether a USB device exists. Drive and steering use `TEL DRIVE` / `TEL STEER` at 115200 baud; ASSI reports use 9600 baud. Never assign roles from `ttyUSB0` or `ttyACM0` numbering.

## Files on the Host

| Path | Purpose |
| --- | --- |
| `/etc/ft-rigby/rigby.toml` | Routine route, operational limits and timeouts |
| `/etc/ft-rigby/controller-firmware.toml` | Managed board/sketch settings and compiled hard stops |
| `/etc/ft-rigby/devices.json` | Expected device configuration |
| `/var/lib/ft-rigby/devices.json` | Enrolled controller identities |
| `/var/lib/ft-rigby/firmware-state.json` | Firmware synchronization record |
| `/run/ft-rigby/status.json` | Runtime status snapshot |

Keep live machine files and unique device identities out of public examples. A replacement board needs deliberate identification and enrolment, not a copied tty path.

## Current Defaults and Recorded Settings

The repository example selects `peak_can`, steering inversion on, ASSI bypass off, reverse off and remote disabled.

| Setting | Example / recorded value |
| --- | --- |
| Main speed cap | 0.15 m/s |
| Main steering cap | Example: +/-8 degrees; host audit: +/-7.5 degrees |
| Drive/steering telemetry age | 0.250 s |
| ASSI report age | 0.500 s |
| ROS command age | 0.500 s |
| CAN command / handshake age | 0.100 s each |
| Device verification window | 5 s |
| Remote speed cap / input age | 0.05 m/s / 0.250 s |

The host values above are the **22 September audit settings**, not a live read of today's vehicle. Read the actual files before a session. The independent compiled firmware limits are covered in [firmware](firmware.md).

Routine TOML changes do not themselves require a controller reflash, but restarting the gate can still run its firmware-sync dependency. Review both files and the intended source revision before restarting.

## Start With Read-Only Checks

On the host:

```bash
systemctl status rigby-device-gate.service --no-pager
journalctl -u rigby-device-gate.service -n 100 --no-pager
cat /run/ft-rigby/status.json
```

These inspect the service and its reports without issuing a movement request. Check timestamps: an old status file is not current telemetry. If the service refuses to start, read the specific error rather than bypassing the gate.

## Choosing a Diagnostic

| Tool | Important effect |
| --- | --- |
| Unit tests | Software-only fixtures; no vehicle needed |
| `remote_hid` input checker | Reads the gamepad, not the actuator ports |
| `rigby-demo --check` | Takes normal control out of service and sends STOP to the identified controllers |
| `rigby-sequence --check` | Opens the controllers and ASSI, then checks STOP states; not passive |
| `rigby-sequence --no-assi --check` | Explicit STOP-only check without opening ASSI |
| `rigby-sequence` | A supervised motion sequence, not an inspection command |
| Firmware/upload tools | May reset or program a board; not ordinary diagnostics |

Opening a serial port can reset a board. Use one serial owner at a time, isolate motor power for STOP-only checks, and read the repository's [diagnostic guide](https://github.com/FT-Autonomous/FT-Rigby/blob/main/docs/DIAGNOSTICS.md) before running a tool.

The motion sequence requests -7 degrees, +7 degrees, centre, then 0.15 m/s forward for three seconds. Its current steering target window is **20 seconds**. It leaves normal control stopped afterwards. `--no-assi` is an explicit diagnostic exception, not the normal way to operate Rigby; the [testing page](../testing.md) records the incomplete powered retest.

## When Something Stops Working

Check power state, physical connections, identity, telemetry and logs in that order before changing configuration. A data-capable replacement USB cable restored the silent steering UNO in the latest connection check. That does not make cable replacement a universal fix.

Do not clear a latched emergency state merely by repeatedly restarting, weaken freshness deadlines, or enable ASSI bypass to get past an unexplained refusal. Resolve the cause and plan the next test with the person responsible for the vehicle.

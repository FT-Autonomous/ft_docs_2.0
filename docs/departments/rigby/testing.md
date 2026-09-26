# Testing and Current Status

This page is the handover for ongoing Rigby work, reconciled to **26 September 2026**. It separates things we have demonstrated from things that merely compile, fit in CAD or pass simulated tests.

!!! warning "Not a ground-operation sign-off"
    The September audit found unresolved control faults. The later USB-cable fix and manual diagnostic did not close them. Keep actuator power isolated for software and wiring checks, and use a supervised setup with independent power removal for any deliberately authorised motion test.

## Recent Results

| Area | What was established | Still not established |
| --- | --- | --- |
| Mechanical build | Platey/holders fitted; drive and steering demonstrated independently | A complete current as-built CAD and wiring record |
| Controller connection | Replacing the UNO USB cable restored steering telemetry; a three-board STOP-only check saw both actuator outputs disabled (23 September) | Reliable behaviour through every reset, reconnect and power cycle |
| Manual sequence | A supervised steering/drive diagnostic exists, including explicit no-ASSI mode | A complete powered rerun after extending the steering timeout |
| Steering timeout | Changed from 4 to **20 seconds** in `72cd96b`; simulation and installed-file checks completed | Physical validation of the longer timeout |
| Gearbox | R4 geometry rebuilt and checked; print prepared/started | Dry fit, torque transfer, backlash and service life |
| CAN and ROS | Isolated route tests passed with simulated serial endpoints | Final physical Jetson/CAN/Ethernet acceptance |
| Remote input | Stadia/DualSense support and the separate ASSI remote mission implemented | Reliable Bluetooth freshness, range and vehicle commissioning |
| Repository | Maintained software published at `a54a513` | Publication of the unpushed Hardware CAD and local R4 files |

The manual sequence's repository guide still describes the older four-second steering window. The executable at the checked revision uses **20 seconds**. Follow the code-backed value here; do not change speed or other limits just to get a diagnostic to finish.

## Controls Audit: Open Findings

The [22 September audit, with its 23 September update](https://github.com/FT-Autonomous/FT-Rigby/blob/a54a513b36a5c12493ab98fe046b5f7f1a33e2b1/docs/FULL_STACK_AUDIT_2026-09-22.md) is the detailed report. Subsequent commits added diagnostics, not fixes to the affected runtime modules.

| Finding | Why it matters |
| --- | --- |
| CAN E-stop overwritten within a receive batch | A later status frame can replace an earlier stop request before the state machine evaluates it. |
| Sequential stop writes | Some paths may not attempt the steering stop if the drive write raises an exception. |
| ASSI receive loop and framing | An unbounded drain can delay other work; partial serial fragments can be accepted as complete state reports. |
| Legacy ASSI DRIVING state | The normal report can fall from `D` back to `R` immediately after GO. |
| ROS command age | Adjustable ROS time can make an old request appear fresh after a backward clock jump. |
| Steering on loss of authority | Some ROS paths actively centre steering rather than disabling it; the stop policy needs resolving. |

There are also feedback-failure gaps. A disconnected/out-of-range potentiometer can be clamped to an apparent endpoint, and absent encoder pulses can appear as zero speed while duty remains applied. Command caps and communication watchdogs do not independently prove physical speed or sensor health.

The audit's earlier silent-UNO result was superseded by the cable replacement. Do not keep diagnosing that old observation as the current fault, but do not use its resolution to dismiss the software findings.

## Test Results Are Bounded

The recorded audit ran 255 host tests and 19 isolated route scenarios: 12 CAN and 7 ROS. The later local suite recorded 272 tests with two skips. Those suites did not cover all of the audit's targeted fault cases.

The latest audit window had no physical PEAK interface and no Ethernet carrier to the Jetson. Passing virtual CAN and local ROS tests therefore did not validate the final vehicle wiring. Likewise, a Bluetooth pairing or a brief healthy input window did not prove continuous fresh control reports.

The exact tests, fixture setup and limitations belong with the result. "Tests passed" by itself is not enough to authorise a drive.

## Next Work, in Order

1. Add regressions for the reproduced control faults and agree the steering stop policy, then fix and review the affected code.
2. Define sensor-failure behaviour before changing firmware or relying on feedback limits.
3. Complete the R4 dry fit and mechanical inspection; record the actual pin geometry and coupling behaviour.
4. Recheck controller identities, wiring, calibration and STOP behaviour with motor power isolated.
5. Run the physical CAN and ROS acceptance checks with the actual Jetson and adapters in a controlled setup.
6. Revalidate steering travel and low-speed motion under supervision. Record physical stopping behaviour separately from a software STOP timestamp.
7. Commission remote control only after input freshness and stop/re-arm behaviour are reliable.

Use the repository's [integration acceptance guide](https://github.com/FT-Autonomous/FT-Rigby/blob/main/docs/INTEGRATION_ACCEPTANCE.md) for the detailed checks. Do not bypass a failed gate, lengthen watchdogs or run a different input mode merely to make the vehicle move.

## What to Record After a Session

Record the software revision, fitted boards and firmware, configuration changes, power arrangement, mechanical revision, test setup, observed result and remaining fault. Keep raw logs and private device details out of the public docs; add a short, reproducible result here.

A fix should replace the corresponding open status only once the relevant test has passed. CAD inspection, STOP-only checks, simulation and loaded motion are separate results.

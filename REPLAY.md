# Match Logging & Replay

The robot logs every match to a WPILOG via AdvantageKit. Use it to scrub the match in AdvantageScope
and to deterministically re-run the logged data through the code.

## What is logged

- **Inputs (replay-deterministic):** every subsystem logs its IO inputs via `@AutoLog` +
  `Logger.processInputs(...)` — `Drive` (per-module), `Gyro`, `Shooter`, `Transfer`, `Intake`, `Pivot` —
  so their control code re-runs exactly in replay.
- **Outputs (review):** `Drive/Pose`, `Drive/ModuleStates`, `Drive/ChassisSpeeds`, `Drive/GyroYaw`,
  `Drive/LockOn/*`, the four `Shooter/*RPS`, `Transfer/Intake/Pivot` state, `Vision/BestTag`, and
  `Auto/Command`. AdvantageKit also auto-logs DriverStation (joysticks, enable, match info),
  NetworkTables, and power.

## 1. Capture (during matches)

- Plug a **FAT32-formatted USB stick** into the roboRIO USB port. `WPILOGWriter()` writes to
  `/U/logs/*.wpilog` automatically (falls back to `/home/lvuser/logs` if no stick — onboard space is
  limited, so prefer the stick).
- Each enable produces a timestamped `.wpilog`.

## 2. Review (post-match)

- Copy the `.wpilog` off the stick (or download via AdvantageScope while connected).
- Open it in **AdvantageScope** → drag fields onto:
  - **2D/3D Field:** `Drive/Pose` to see the robot drive the match.
  - **Line graphs:** `Shooter/*RPS`, `Transfer/Intake/Pivot` state, `Drive/ChassisSpeeds`.
  - **Timeline:** `Auto/Command` and the auto-logged DS/command data for the decision sequence.

## 3. Deterministic re-run (test code changes on real data)

The IO subsystems (Transfer/Intake/Pivot) re-run exactly against the logged inputs:

1. Set `REPLAY = true` in `Robot.java`.
2. Make the log discoverable by `LogFileUtil.findReplayLog()` (drop it in the project root, or use
   AdvantageScope's *File → Export Replay Log*).
3. Run `./gradlew simulateJava` — or `./gradlew replayWatch` to auto-rerun on every code edit.
   It reads the log, re-runs the code, and writes `<log>_sim.wpilog` with the recomputed outputs.
4. Compare the original vs `_sim` outputs in AdvantageScope.
5. Set `REPLAY = false` again when done.

> `Drive` (modules), `Gyro`, `Shooter`, `Transfer`, `Intake`, and `Pivot` all recompute exactly in
> replay through their logged IO inputs. Only `Vision` is review-only — it is real-camera-only with no
> input layer, so in replay it produces no new measurements (its logged `Vision/BestTag` is for review).

## Notes

- To produce a log from **simulation** for testing replay, temporarily add
  `Logger.addDataReceiver(new WPILOGWriter("logs"))` to the SIM branch in `Robot.java`, run
  `simulateJava`, then revert.
- `Robot.java` records `RobotMode` (`REAL`/`SIM`/`REPLAY`) metadata so you can tell logs apart.

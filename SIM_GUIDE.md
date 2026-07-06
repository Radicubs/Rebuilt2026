# Simulation Guide

How to run, drive, and visualize the robot in simulation from a laptop. No hardware needed.
(For match logging & post-match replay, see `REPLAY.md`.)

---

## 1. Launch

From the project root:

```bash
./gradlew simulateJava
```

(If you get "permission denied", use `sh gradlew simulateJava`. In VS Code you can instead run
`Cmd/Ctrl+Shift+P → "WPILib: Simulate Robot Code"` → tick **Sim GUI**.)

Wait for the terminal to print `********** Robot program startup complete **********`. A window
titled **"Robot Simulation"** opens with several panels (Robot State, System Joysticks, Joysticks,
NetworkTables, Timing, …).

---

## 2. Fastest win — run an autonomous (no controller needed)

1. **NetworkTables** panel → `SmartDashboard` → **Auto Chooser** → pick e.g. *Left Shoot*.
2. **DS** panel → set the **Alliance** (Blue/Red) so PathPlanner mirrors paths correctly.
3. **Robot State** panel → click **Autonomous**. The robot is enabled and runs the auto.

Add visualization (next section) to actually watch it drive the path.

### Running another auto (without restarting)

1. **Robot State → Disabled** (stops the current/finished auto).
2. **Auto Chooser** → pick a different auto.
3. **Robot State → Autonomous** again.

`Robot.autonomousInit()` re-fetches the selected auto each time you enter Autonomous, and PathPlanner
resets the pose to that auto's starting point — so it just runs the new one. For a totally clean field
(e.g. things look off), just restart `./gradlew simulateJava`.

### "Why did my robot slowly drift on its own? (fixed)"

The pose used to slide on its own after autos. The cause was the **vision simulation** feeding the
fused (vision-corrected) pose back into the simulated cameras — a feedback loop that dragged the pose.
Since the simulated gyro makes the wheel+gyro odometry exact, vision sim was unnecessary, so it was
**removed**. The sim pose is now pure wheel+gyro odometry — it does not drift. (The real-robot vision
code is untouched; it just produces no measurements in sim.)

---

## 3. Visualization (AdvantageScope — ships with WPILib)

1. Open AdvantageScope: VS Code `Cmd/Ctrl+Shift+P → "WPILib: Start Tool" → AdvantageScope`
   (or launch from `~/wpilib/2026/tools/`).
2. Menu **File → Connect to Simulator** (connects to `localhost`). The left sidebar fills with live data.
3. Add a **2D Field** tab (`+`), then drag **`AdvantageKit → RealOutputs → Drive → Pose`** onto it.
   Pick the field image. Re-run an auto → watch the robot drive.
4. Add a **Line Graph** tab and drag `RealOutputs/Shooter/LeftRPS`, `Pivot/Position`,
   `Transfer/SpeedRPS`, `Intake/VelocityRPS` to graph mechanisms over time.

Everything under `AdvantageKit/RealOutputs/...` is what we log each loop (same data that's written to
match logs for replay). **Lighter fallback:** Glass (`Start Tool → Glass`) → `NetworkTables → Connect`
→ `SmartDashboard → Field`.

**Recommended layout (set up once):**
- **Tab "Field" (2D Field):** `RealOutputs/Drive/Pose`.
- **Tab "Drive" (Line Graph):** `RealOutputs/Drive/ChassisSpeeds`, `Drive/GyroYaw`, `Drive/LockOn/ErrorRad`.
- **Tab "Mechanisms" (Line Graph):** `Shooter/LeftRPS`, `Shooter/RightRPS`, `Pivot/Position`,
  `Transfer/SpeedRPS`, `Intake/VelocityRPS`.

**One-click next time:** AdvantageScope **auto-restores your last layout** when you reopen it, so you
only do this once. Use **File → Save Layout** to keep a named backup you (or teammates) can re-import.

---

## 4. Driving — Option A: USB controller (easiest, full functionality)

1. Plug an Xbox/PS-style controller into the laptop.
2. In the sim window, the **System Joysticks** panel lists it → **drag it into the Joysticks panel,
   slot 0** (driver). Drag a second controller (or the same approach) into **slot 1** (operator).
3. **Robot State → Teleoperated**. Drive with the sticks; all buttons/triggers in the table below work.

This is the recommended setup — no key mapping, and the trigger-based shots work.

---

## 5. Driving — Option B: keyboard (laptop-only)

The sim turns your keyboard into virtual joysticks. Driver = **Joystick 0**, Operator = **Joystick 1**.

### 5a. Map the driver keyboard
1. Menu bar **DS → Keyboard 0 Settings**. Each **Axis** row has a *decrease* and *increase* key; each
   **Button** row has one key. Click a key box and press the key to bind.
2. Bind:

| Robot action | Axis / Button | Keys |
|---|---|---|
| Forward / back | Axis 1 (left Y) | `W` decrease · `S` increase |
| Strafe | Axis 0 (left X) | `A` decrease · `D` increase |
| Rotate | Axis 4 (right X) | `Q` decrease · `E` increase |
| Shoot (indexer+transfer) | Button 6 (RB) | `Space` |
| Zero heading | Button 5 (LB) | `Z` |
| Flip heading 180° | POV Down | bind POV "down" (e.g. `↓`) |

3. **System Joysticks → drag "Keyboard 0" into Joysticks slot 0.**

### 5b. Map the operator keyboard
1. Menu bar **DS → Keyboard 1 Settings**. Bind:

| Robot action | Button | Suggested key |
|---|---|---|
| Intake + extend pivot | Button 3 (X) | `I` |
| Close-shot ramp | Button 6 (RB) | `R` |
| Pivot DOWN | Button 1 (A) | `G` |
| Pivot UP | Button 2 (B) | `T` |
| Reset pivot angle | Button 7 (Back) | `C` |
| Regression shoot | Button 4 (Y) | `F` |

2. **Drag "Keyboard 1" into Joysticks slot 1.**

3. **Robot State → Teleoperated**, click the sim window for focus, and drive/press keys.

> The **trench** and **pass** ramps use the controller *triggers* (held analog), which are impractical
> on a keyboard — use a USB controller for those. Everything else works from keys.

---

## 6. Control bindings reference (from `RobotContainer`)

**Driver (Joystick 0)** — Xbox button #s in parentheses:
| Control | Action |
|---|---|
| Left stick | Translate (forward / strafe) |
| Right stick X | Rotate |
| Right bumper (6), held | Shoot — indexer + transfer |
| Left bumper (5) | Zero heading |
| D-pad Down | Flip heading 180° |
| X (3) | Toggle **lock-on** — rotates to aim at the hub (see below) |

**Lock-on in sim:** set the **Alliance** in the DS panel first (lock-on needs one), then press **X**
to toggle. The robot rotates to face the hub using the odometry pose + known hub location — no cameras
needed, and the sim pose is exact so it aims true. Moving the rotation stick cancels it. Watch
`RealOutputs/Drive/LockOn/Active` and `.../ErrorRad` in AdvantageScope.

**Operator (Joystick 1):**
| Control | Action |
|---|---|
| X (3), held | Intake + extend pivot (if pivot idle) |
| Right bumper (6), held | Close-shot ramp |
| Right trigger, held | Trench ramp |
| Left trigger, held | Pass ramp |
| A (1) | Pivot to down position |
| B (2) | Pivot to up position |
| Back (7) | Reset pivot angle |
| Left stick (9), held | Manual pivot |
| Y (4), held | Regression shooting |
| D-pad Up/Down | Adjust main-shooter custom speed ± |
| D-pad Left/Right | Adjust top-shooter custom speed ± |

---

## 7. Pro tips

- **Layouts persist.** The sim GUI auto-saves your panel layout + keyboard maps on exit, so sections 4–5
  are a one-time setup. In AdvantageScope, **File → Save Layout** to keep your field + graphs.
- **Set Alliance** before running autos (DS panel) — PathPlanner mirrors red/blue.
- **Step through time:** the **Timing** panel lets you pause / single-step the sim to inspect a moment.
- **Sim is accurate enough to trust:** drivetrain speeds, rotation, shooter, and all 8 autos track their
  planned paths closely; the sim auto-selects every simulated subsystem because
  `RobotBase.isSimulation()` is true under `simulateJava`.
- **Post-match replay & logging:** see `REPLAY.md`.

---

## 8. How the simulation is organized

Every subsystem uses the same **IO-layer pattern** — there is no central sim manager, and sim code never
runs on the real robot. Each subsystem holds an `XIO` interface and picks the implementation once via
`RobotBase.isSimulation()`; its control logic and public API are identical in both modes, and all
**vendor hardware lives only in the `*IOReal` files**:

- **Drive** — `ModuleIO` (`ModuleIOReal` = TalonFX on-device velocity/position closed loops + CANcoder;
  `ModuleIOSim` = WPILib `DCMotorSim` driven by Java closed loops with the same gains). Odometry,
  PathPlanner, and the pose estimator run identically in both modes.
- **Gyro** — its own subsystem: `GyroIO`/`GyroIOReal` (NavX) / `GyroIOSim` (yaw integrated from the
  drive kinematics in sim).
- **Shooter** — `ShooterIO` (`ShooterIOReal` = four TalonFX velocity flywheels; `ShooterIOSim` = four
  WPILib `FlywheelSim` with a Java velocity controller).
- **Intake / Transfer / Pivot** — `XIO` (`XIOReal` = SparkMax; `XIOSim` = WPILib `FlywheelSim`/`DCMotorSim`).
- **Vision** — **real-only, no sim.** Simulated localization is the exact wheel + gyro odometry, so the
  pose is precise and never drifts, and lock-on (which aims from the pose) works without cameras. The
  real `Vision` pipeline is intact; it simply produces no measurements in sim.

Every subsystem logs its IO **inputs** via `@AutoLog` + `Logger.processInputs(...)`, so the whole robot
is inspectable in AdvantageScope and replay-ready. Sim constants (moments of inertia, etc.) live in each
subsystem's `*Constants.SimConstants` and affect only sim feel, never the real robot.

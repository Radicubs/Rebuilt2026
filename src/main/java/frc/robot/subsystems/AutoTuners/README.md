# Porting AutoTuners to another robot project

These are standalone, command-based WPILib Java subsystems for identifying and
testing flywheel, arm, and elevator gains. Each mechanism can be copied separately.
The core tuners use WPILib; motor-specific access is supplied through an `IO`
interface. The included simulation mains exercise the same production algorithms.

For configuration fields, equations, units, and mechanism-specific behavior, see
[Flywheels](Flywheels/README.md), [Arms](Arms/README.md), and
[Elevator](Elevator/README.md).

## 1. Choose what to copy

Copy this folder into the destination project's
`src/main/java/frc/robot/subsystems/`, preserving this layout:

```text
AutoTuners/
  README.md
  run.ps1
  tuner-simulation.gradle
  Flywheels/
    FlywheelAutoTuner.java
    FlywheelAutoTunerIOSim.java
    FlywheelTunerSimulation.java
    FlywheelAutoTunerTalonFXIO.java
    README.md
  Arms/
    ArmAutoTuner.java
    ArmAutoTunerIOSim.java
    ArmTunerSimulation.java
    README.md
  Elevator/
    ElevatorAutoTuner.java
    ElevatorAutoTunerIOSim.java
    ElevatorTunerSimulation.java
    README.md
```

For real hardware, only the chosen `*AutoTuner.java` and your IO implementation
are required. For its automated simulation, also copy that mechanism's
`*AutoTunerIOSim.java`, `*TunerSimulation.java`, and the shared Gradle helper.
`run.ps1` is an optional convenience runner. Keep the helper at the folder root;
it locates the three mechanism files by these exact folder/file names.

`FlywheelAutoTunerTalonFXIO.java` requires Phoenix 6 even when it is merely present
in your normal Java source tree. Omit it in projects without Phoenix, or implement
the flywheel IO for your controller instead. The other tuner/simulation files do
not require Phoenix, REVLib, AdvantageKit, or this robot's constants/subsystems.
Do not copy this project's `RobotContainer` or its CAN IDs.

Keep the package declarations and imports together:

```java
import frc.robot.subsystems.AutoTuners.Flywheels.FlywheelAutoTuner;
import frc.robot.subsystems.AutoTuners.Arms.ArmAutoTuner;
import frc.robot.subsystems.AutoTuners.Elevator.ElevatorAutoTuner;
```

If your robot uses a different package, update declarations, imports, and source
paths consistently. The helper reads the simulation main's declared package,
but still expects the mechanism folders and filenames above.

## 2. Set up Gradle

Start with the destination project's WPILib Java command-based GradleRIO setup.
Keep its wrapper, team number, deployment configuration, and season-compatible
dependency versions. This source was compiled in this repository with
GradleRIO `2026.2.1`, Gradle `8.14`, and Java 17 source/target compatibility.
Other combinations need validation in the destination project. The simulation
helper explicitly compiles with `options.release.set(17)`; run Gradle with a
JDK compatible with your Gradle/GradleRIO versions and capable of compiling Java 17.

Check the following entries in the existing `build.gradle`; merge missing entries
into its existing blocks instead of duplicating them or replacing the build file:

```groovy
java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

dependencies {
    implementation wpi.java.deps.wpilib()
    implementation wpi.java.vendor.java()

    // Real roboRIO execution, normally already in the WPILib template:
    roborioDebug wpi.java.deps.wpilibJniDebug(wpi.platforms.roborio)
    roborioRelease wpi.java.deps.wpilibJniRelease(wpi.platforms.roborio)

    // Required by the included standalone simulation tasks:
    nativeRelease wpi.java.deps.wpilibJniRelease(wpi.platforms.desktop)

    // Also keep these for the normal WPILib desktop simulator:
    nativeDebug wpi.java.deps.wpilibJniDebug(wpi.platforms.desktop)
    simulationDebug wpi.sim.enableDebug()
    simulationRelease wpi.sim.enableRelease()
}

// For the normal interactive robot simulator:
wpi.sim.addGui().defaultEnabled = true
wpi.sim.addDriverstation()
```

The command framework (`edu.wpi.first.wpilibj2.command`) must also be available.
Retain the command-based template's WPILib New Commands vendordep if that is how
your project supplies it. For vendor hardware, install the destination-season
vendor vendordep and retain its generated Java/JNI dependency entries, including
roboRIO natives and desktop natives when that vendor is used in desktop simulation.
The generic tuner cores and physics fixtures need only WPILib and commands.

Keep the normal GradleRIO `jar`, deployment, and executable-task configuration
from your robot template. The snippet above is a dependency checklist, not a
complete `build.gradle`.

Do **not** add `tuner-simulation.gradle` using `apply from:`. It is an init script
loaded explicitly with `--init-script`. You do not need to register its tasks in
`build.gradle`. It registers a mechanism's tasks only when all three of that
mechanism's Java files exist.

## 3. Run the automated physics simulations

Run these commands from the destination project root. Windows PowerShell:

```powershell
# All three mechanisms; copy all three folders for this default:
./src/main/java/frc/robot/subsystems/AutoTuners/run.ps1

# Choose one, including when only that mechanism was copied:
./src/main/java/frc/robot/subsystems/AutoTuners/run.ps1 -Kind Flywheel
./src/main/java/frc/robot/subsystems/AutoTuners/run.ps1 -Kind Arm
./src/main/java/frc/robot/subsystems/AutoTuners/run.ps1 -Kind Elevator

# Explicit environment overrides; replace these paths with your own:
./src/main/java/frc/robot/subsystems/AutoTuners/run.ps1 -Kind Arm -ProjectRoot C:/Robots/MyRobot -Jdk C:/Users/Public/wpilib/2026/jdk -Offline
```

The runner searches upward for the Gradle wrapper, honors `JAVA_HOME`, and can
discover a standard WPILib JDK installation. `-Jdk` overrides Java selection.
Use `-Offline` only after the wrapper and dependencies are cached. It restores
the calling directory and `JAVA_HOME` on exit and saves stdout under
`build/flywheel-sim/results.txt`, `build/arm-sim/results.txt`, or
`build/elevator-sim/results.txt`. Read stderr for Gradle/assertion failures too.

Without the PowerShell runner, invoke the helper directly:

```powershell
./gradlew.bat --init-script src/main/java/frc/robot/subsystems/AutoTuners/tuner-simulation.gradle flywheelTunerSim armTunerSim elevatorTunerSim
```

On macOS/Linux, use `./gradlew` in place of `./gradlew.bat`. Select only the tasks
you copied, and append `--offline` if appropriate. The helper currently assumes
a single root Java GradleRIO project and standard `build/jni/release` natives.

Each task compiles only its tuner and simulation IO/test pair into
`build/portable-tuners/<kind>/classes`, extracts release desktop natives, and runs
with the destination project's runtime dependencies. It does not instantiate
`RobotContainer`, deploy code, or connect to hardware. Consequently, passing a
suite does not establish that the destination robot source compiles; also run:

```powershell
./gradlew.bat compileJava
```

The mains initialize HAL, enable simulated Test mode, and step physics/time at
20 ms. They perform identification, candidate tracking, disturbances, and fault
checks, then print `RESULT PASS` and `BUILD SUCCESSFUL`.

| Suite | Current expected checks | Coverage fixtures |
| --- | ---: | --- |
| Flywheel | 2,023 | Three inertia cases |
| Arm | 7,782 | Five cases plus stalled motion |
| Elevator | 7,847 | Five cases plus stalled motion |

To model your mechanism, edit the physical parameters, tuner configuration,
targets, and acceptance tolerances together in its `*TunerSimulation.java`.
Fixture limits and bootstrap gains are examples, not measured hardware settings.

### Interactive robot simulation

`simulateJava` runs your robot program; the standalone tasks above run their own
test mains. To use a tuner in the robot simulator, construct its simulated IO
instead of hardware IO, register the tuner normally, and advance the IO exactly
once per 20 ms before the tuner reads it. In a scheduler integration, update the
plant at the start of `robotPeriodic()` before `CommandScheduler.run()`. Let the
scheduler call the tuner's `periodic()`; do not call it a second time yourself.
Keep that plant update separate from any existing update in `simulationPeriodic()`.

Launch with `./gradlew.bat simulateJava`, select and enable Test mode in the
simulated Driver Station, and use your tuning bindings described below. The
normal desktop simulator manages time/HAL; do not also copy the test main's
`SimHooks.pauseTiming()` or time-stepping loop into your robot program.

## 4. Connect real hardware IO

Reuse the existing motor controller object. Do not construct another controller
with the same CAN ID for the tuner. Read fresh mechanism measurements and apply
voltage directly; pause any controller that would overwrite that voltage.

| IO | Values returned by `readInputs()` | Positive direction |
| --- | --- | --- |
| `FlywheelAutoTuner.IO` | `Inputs(velocityRPS, appliedVolts, currentAmps, connected)` | Positive wheel speed |
| `ArmAutoTuner.IO` | `Inputs(angleRad, velocityRadPerSec, appliedVolts, currentAmps, connected)` | Increasing angle, zero at horizontal |
| `ElevatorAutoTuner.IO` | `Inputs(heightMeters, velocityMetersPerSec, appliedVolts, currentAmps, connected, homed, lowerLimit, upperLimit)` | Increasing carriage height |

Each interface also requires `void setVoltage(double volts)`. `appliedVolts`
must describe the preceding control interval. Convert encoder units/gearing
once, make current units match the configured threshold, and make `connected`
include sensor validity/freshness. Configure hardware current limiting separately.
Arm and elevator hardware adapters are project-specific and are not included.

For a Phoenix 6 flywheel, the included adapter can be wired as follows. The
variables in this example must come from your existing subsystem and measured
mechanism limits:

```java
import frc.robot.subsystems.AutoTuners.Flywheels.FlywheelAutoTuner;
import frc.robot.subsystems.AutoTuners.Flywheels.FlywheelAutoTunerTalonFXIO;

// Inside your subsystem/container initialization:
var config = new FlywheelAutoTuner.Config(
    maxRPS, maxSupplyAmps, maxRunVolts, lowTestVolts, highTestVolts, bandwidth);
var io = new FlywheelAutoTunerTalonFXIO(
    existingTalonFX, motorTurnsPerWheelTurn, config.maxCurrentAmps());
var tuner = new FlywheelAutoTuner(io, config, "ShooterTuner");
```

The adapter enables/applies the supplied hardware supply-current limit at
construction. It does not choose the CAN ID, bus, inversion, or signal rates.
Verify those in your project. Its signal-status checks do not independently
audit timestamps. A flywheel's default config is only an example; arms and
elevators require explicit configurations. Follow the linked mechanism guides.

## 5. Add Test-mode controls and lifecycle cleanup

Keep `CommandScheduler.getInstance().run()` executing once every 20 ms in
`robotPeriodic()`, including Test mode. The tuner extends `SubsystemBase` and
registers itself with the scheduler. Do not also invoke its `periodic()` manually.

Create two separate hold-to-run bindings. This flywheel example assumes `tuner`,
`mechanismSubsystem`, and `testController` are fields in your container, and
`safeTargetRPS` is a mechanism-specific configured value:

```java
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;

testController.x().and(DriverStation::isTestEnabled).whileTrue(
    Commands.startEnd(tuner::beginTuning, tuner::stop, tuner, mechanismSubsystem));

testController.y().and(DriverStation::isTestEnabled).whileTrue(
    Commands.startEnd(() -> tuner.runCandidate(safeTargetRPS), tuner::stop,
        tuner, mechanismSubsystem));
```

For an arm, replace the target with a safe angle in radians; for an elevator,
use a safe carriage height in meters. Hold calibration until identification
finishes, release, then hold the separate candidate button. `beginTuning()` must
be called once per attempt, not continuously in `periodic()` or `Commands.run()`.

Both commands require the normal mechanism subsystem to exclude competing
commands while held. Requirements alone do not stop a controller inside
`periodic()` or a default command between tests. Add an explicit tuning ownership
mode to the normal subsystem: while it is active, suppress normal PID, default
command output, and any other motor writes for the entire tuning session.
Allow the tuner's IO to write voltage through a dedicated path. Do not restore
normal control just because a calibration button was released.

In your existing robot/container lifecycle methods, arrange the following;
these are actions to integrate with your project's own method names:

| Lifecycle | Actions |
| --- | --- |
| Enter Test | Cancel existing commands, pause normal mechanism control, stop tuner output, then allow tuning bindings |
| Disable | Call `stop()` on every tuner immediately; keep normal control suppressed while disabled |
| Leave Test | Cancel tuning commands and call every tuner's `stop()`; restore normal ownership only when the next enabled mode is prepared |

The cores also stop active output when Test is no longer enabled, but explicit
cleanup makes command interruption and ownership handoff clear. For gravity
mechanisms, stopping applies zero volts and does not hold the load.

## 6. Deploy and tune the real mechanism

1. Configure the destination team number and vendor dependencies, then build
   and deploy using its normal workflow (`./gradlew.bat build` and
   `./gradlew.bat deploy`). Simulation tasks are not required for deployment.
2. Verify sensor units, sign, reference, current limits, and exclusive motor
   ownership before beginning. Establish the arm's horizontal reference; home
   the elevator using its normal procedure. Choose safe physical travel and
   provisions for supporting/braking gravity mechanisms when output stops.
3. Start stopped: at most 2 RPS for flywheels, 0.1 rad/s for arms, or 0.1 m/s
   for elevators. Arm/elevator starting clearance to each bound must be at
   least `sweepAmplitude + 2*limitMargin`, in the appropriate units; elevator
   switches must be released and `homed` must be true.
4. Select **Test** in Driver Station, enable, and hold the calibration button.
   Flywheels collect for about 5 seconds; arms/elevators collect for 12 seconds.
   Output stops at completion. Watch `<prefix>/TuneStatus` and `TuneSamples`.
5. Wait for `Candidate gains ready` and a nonnull `getCandidate()`. Release
   calibration, then hold the candidate-test button. Observe speed/position,
   current, output voltage, and tracking. Release or disable to stop.
6. Record the candidate gains deliberately. They are held in memory and
   published to dashboard keys; they are not saved to constants or motor
   firmware automatically. Convert units and control-loop conventions before
   using them in the normal controller, then validate normal operation.

Flywheel targets must be positive, within `maxTestRPS`, with feedforward no more
than `maxRunVolts - 1 V`. Arm/elevator targets must be strictly inside each bound
by twice the margin. Arm/elevator bootstrap gains must already be appropriate
for collecting a bounded sweep. Simulation success does not establish safe
bootstrap gains or physical-robot performance.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Java or Gradle startup failure | Match the destination wrapper/GradleRIO and JDK; set `-Jdk` or `JAVA_HOME` |
| Wrapper download despite `--offline` | The wrapper distribution itself must already be cached; offline dependency mode does not install it |
| `wpilibj2.command` missing | Restore the command framework dependency/vendordep from the destination command-based template |
| Phoenix import errors | Install matching Phoenix 6 or remove the optional TalonFX adapter from the copied sources |
| Simulation task missing | Pass `--init-script`; check helper placement and all three mechanism filenames |
| JNI/library load error | Check release desktop natives and `extractReleaseNative`; customized native paths need helper changes |
| Simulation passes but robot fails to compile | Run `compileJava`; isolated tasks intentionally omit unrelated robot sources |
| No motion / requires Test mode | Enable Test, hold calibration, verify scheduling and exclusive IO ownership |
| Not enough samples / bad timing | Keep the 20 ms loop, fresh voltage/sensor readings, and avoid blocking robot work |
| Poor fit or insufficient motion | Check units, references, polarity, voltage reporting, and the mechanism guide's fit requirements |
| Active safety abort | Fix the reported sensor, current, speed, travel, switch, or homing condition before restarting |

Do not increase excitation or travel beyond mechanism limits to force a fit.
An active safety abort clears candidate gains; a normal `stop()` retains them.

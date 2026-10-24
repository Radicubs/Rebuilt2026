# Flywheel tuner — FlywheelAutoTuner

This folder contains `FlywheelAutoTuner.java` (the standalone WPILib tuner), `FlywheelAutoTunerIOSim.java` (physics IO), and `FlywheelTunerSimulation.java` (integration checks). `FlywheelAutoTunerTalonFXIO.java` is an optional Phoenix 6 adapter.

The parent folder provides the shared `run.ps1` and `tuner-simulation.gradle`.

## Variables to change first

| Variable | Units / purpose |
| --- | --- |
| `maxTestRPS` | Maximum absolute flywheel speed in mechanism rotations/second |
| `maxCurrentAmps` | Current threshold; match your IO's supply/stator current definition. Active abort uses threshold + 2 A |
| `maxRunVolts` | Output limit, greater than 1 and at most 12 V |
| `lowTestVolts`, `highTestVolts` | Identification voltages: `0 < low < high <= maxRunVolts` |
| `bandwidthRadPerSec` | Pole-placement parameter `lambda`, greater than 0 and at most 10 |
| IO motor/gearing/sign | Positive voltage must produce positive mechanism RPS |
| `dashboardPrefix` | Optional unique namespace; defaults to `Flywheel` |
| `requestedRPS` passed to `runCandidate` | Positive test speed within the speed and feedforward voltage budget |

`Config.defaults()` means `(75,35,8,2,4,4)`. Replace these example limits as
needed. Current limiting is the hardware owner's responsibility; the generic
core stops on excessive reported current but does not implement a current regulator.

## How to use

Copy `FlywheelAutoTuner.java` with its package folder and import:

```java
import frc.robot.subsystems.AutoTuners.Flywheels.FlywheelAutoTuner;

var config = new FlywheelAutoTuner.Config(maxRPS, maxAmps, maxVolts, lowVolts, highVolts, lambda);
var tuner = new FlywheelAutoTuner(existingFlywheelIO, config, "ShooterTuner");
```

Implement `FlywheelAutoTuner.IO.readInputs()` to return
`new FlywheelAutoTuner.Inputs(mechanismRPS, appliedVolts, currentAmps, connected)`.
`appliedVolts` describes the interval that just elapsed; health must mean fresh
valid measurements. Implement `setVoltage(volts)` using your existing motor.
For a TalonFX, the [optional adapter](#optional-talonfx-adapter) handles rotor
RPS conversion and supply-current configuration.

1. Pause the normal shooter controller and give the tuner exclusive motor ownership.
2. Enable Test mode with the wheel stopped (absolute speed at most 2 RPS).
3. Call `beginTuning()` once; continue the scheduler's 20 ms periodic updates.
4. Wait about five seconds for `getStatus()` to say `Candidate gains ready` and
   `getCandidate()` to become nonnull. Do not repeatedly call `beginTuning()`.
5. Call `runCandidate(requestedRPS)`. Keep Test mode enabled while observing speed.
6. Call `stop()` on command release/interruption, or disable.

For controller bindings, use `Commands.startEnd(tuner::beginTuning, tuner::stop,
tuner, mechanismSubsystem)` for calibration and a separate start/end command
calling `runCandidate` for testing. Holding a calibration button must allow the
entire five-second sequence. Release it before testing. Requirements exclude
other commands, but a normal controller inside the mechanism's `periodic()`
still needs to be paused explicitly.

## What should happen

| Time after begin | Requested voltage |
| --- | --- |
| 0–0.5 s | 0 V |
| 0.5–2.0 s | `lowTestVolts` |
| 2.0–3.5 s | `highTestVolts` |
| 3.5–5.0 s | `lowTestVolts` |
| At completion | 0 V and candidate calculation |

The wheel accelerates at each positive-voltage segment. Gain calculation does
not automatically start a candidate test. A successful test reports
`Testing candidate PI gains`, accelerates toward the target, and holds it until
stopped. Output is limited to `0..maxRunVolts`: it provides no reverse/braking
voltage. Speed can overshoot during acceleration or recoveries.

Dashboard keys include `TuneStatus`, `VelocityRPS`, `TuneSamples`, `kP`, `kI`,
`kD`, `kV`, `a`, `b`, `ModelRMSE`, `TargetRPS`, and `CommandVolts` beneath your
prefix. `getCandidate()` returns the gains; `getSampleCount()` exposes sample
count. `stop()` resets the PI controller and output but retains the candidate;
it does not change the flywheel status string, so use `isRunning()`/output to
check whether a previously displayed test has stopped.

## Failure handling and using the gains

Disabled calibration, movement at start, unsafe sensor/current/speed readings,
insufficient excitation, poor model fit, or nonpositive calculated PI gains
prevent a usable candidate. Active safety abort clears existing gains. Bad
target values stop output. Requested feedforward must fit below
`maxRunVolts - 1 V`; the remaining volt is reserved for feedback. A requested
test during identification is rejected without ending collection.

The fit needs at least 80 samples and observed speed of at least 5 RPS. Samples
must have 15–25 ms timing, unchanged command, positive command, and measured
voltage within 0.4 V of the previous command. If these conditions are unreliable,
check signal rates and applied-voltage reporting before increasing excitation.
If `2*lambda <= a`, this design yields nonpositive kP and rejects the gains;
reconsider bandwidth/mechanism rather than treating a rejected fit as usable.

Gain units: kP V/RPS, kI V/(RPS·s), kD V/(RPS/s), kV V/RPS. The integral
contribution is limited to ±2 V. Export gains deliberately; they are not saved
automatically. Vendor velocity loops may use different units and run periods.
This model omits Coulomb friction and acceleration feedforward.

## Calculation breakdown

For each accepted sample, let `v=previousVelocity`, `u=appliedVolts`, and
`y=nextVelocity`. Fit `y=A*v+B*u` by least squares. Accumulate:

```text
VV=Σv²    VU=Σvu    UU=Σu²    VY=Σvy    UY=Σuy
det=VV*UU-VU²
A=(VY*UU-UY*VU)/det
B=(UY*VV-VY*VU)/det
```

Reject `det <= 0.0001*VV*UU`, `A` outside `(0,1)`, or `B <= 0`.
At sample period `dt=0.020 s`, convert to continuous dynamics:

```text
a=-ln(A)/dt                      [1/s]
b=B*a/(1-A)                      [RPS/(V*s)]
velocityDerivative=-a*velocity+b*volts
RMSE=sqrt(Σ(y-A*v-B*u)²/N)        [RPS]
```

RMSE must not exceed `max(0.30 RPS, 0.03*maxObservedVelocity)`.
The PI feedback characteristic polynomial is
`s²+(a+b*kP)*s+b*kI`. Matching it to `(s+lambda)²` gives:

```text
kP=(2*lambda-a)/b
kI=lambda²/b
kD=0
kV=a/b
```

For example, `a=2.32627`, `b=20.73435`, and `lambda=4` produce approximately
`kP=0.27364`, `kI=0.77167`, `kV=0.112194`. The 20 RPS steady-state feedforward
is `0.112194*20=2.24388 V`.

During a candidate test, `error=targetRPS-measuredRPS` and
`volts=clamp(kV*targetRPS+kP*error+kI*integral(error), 0, maxRunVolts)`.
Voltage/integral limits and 20 ms sampling mean the ideal pole match does not
guarantee the real mechanism's overshoot or settling time.

## Physics simulation IO

### Variables to change first

| Constructor input | Meaning |
| --- | --- |
| `gearbox` | WPILib `DCMotor` model and motor count |
| `inertiaKgM2` | Flywheel mechanism inertia, kg·m², positive |
| `gearing` | Motor rotations per flywheel rotation, positive |
| `supplyVolts` | Available supply, greater than 0 and at most 12 V |

Use a matching `FlywheelAutoTuner.Config` voltage budget. The ideal plant has
no motor-controller current limiter; select test current thresholds accordingly.

### Expected behavior and helper methods

`setVoltage` applies voltage to the physics plant; `readInputs()` returns mechanism
RPS, applied volts, estimated supply current, and valid connectivity. Invalid or
out-of-supply voltage throws `IllegalArgumentException`. `getCommandVolts()`
exposes output. `setVelocityRPS()` injects a load-like speed disturbance.
`setSensorOverride(inputs)` replaces measurements for fault tests; passing null
restores physics readings. No CAN device, robot constants, or vendor JNI is used.

This is a first-order ideal motor/inertia plant without friction, battery sag,
vendor firmware, CAN latency, or hardware current limiting. It validates the
production tuner logic rather than the motor controller implementation.

### Calculation breakdown

WPILib constructs the flywheel linear system from the motor, reduction `G`,
and inertia `J`, then integrates voltage to angular speed. Mechanism speed is
`RPS=angularVelocityRPM/60=angularVelocityRadPerSec/(2*pi)`.
The ideal motor equation gives voltage coefficients in RPS units:

```text
kV=2*pi*G/motor.KvRadPerSecPerVolt
kA=2*pi*J*motor.rOhms/(G*motor.KtNMPerAmp)
```

Reported supply-current magnitude is approximated as
`abs(motorCurrent*appliedVolts/supplyVolts)` from electrical power conversion.
This approximation does not model controller losses or current regulation.

## Simulation checks

### Variables to change first

| Fixture value | Where / default |
| --- | --- |
| Inertias | `inertias`: 0.003, 0.010, 0.001 kg·m² |
| Motor/reduction/supply | Constructor call: one Falcon 500, 1:1, 12 V |
| Tuner limits/excitation/bandwidth | `Config.defaults()`; 0.010 case uses `(65,100,7,1.5,3.5,5)` |
| Test target / duration | 20 RPS, 250 ticks (5 s) |
| Disturbance / recovery | 30% speed loss, 150 ticks (3 s) |
| Acceptance | Final-second and recovery error below 0.2 RPS |

The 100 A fixture threshold allows an ideal voltage-driven plant without a
current regulator; it is not a suggested real-robot limit. If changing speed or
physics, update the expected assertions and config deliberately.

### Expected output and coverage

The process initializes simulated HAL and Test mode, runs three cases, and
prints `IDENTIFY`, `RUN`, `RECOVERY`, then `RESULT PASS checks=2011 cases=3`.
Each ideal case normally identifies 222 samples. Current fixtures have worst
final-second error below 0.00003 RPS and maximum reported peak around 21.79 RPS.
These precise ideal-plant values are not hardware promises; assertions use
the 0.2 RPS tolerance. A failed assertion makes Java/Gradle exit nonzero.

Tests also exercise candidate requirements, invalid/nonfinite targets, voltage
bounds, disabled starts/stops, overspeed, overcurrent, invalid/disconnected
sensors, and candidate invalidation. The core is used directly, without compatibility wrappers, reflection, or Phoenix JNI.
The fixture does not test all hardware faults or real motor-controller firmware.

### Calculation breakdown

For each target tick, `error=abs(measuredRPS-20)`. The final-second metric is
the maximum error over ticks 200–249 of the 250-tick candidate run. Peak RPS is
the maximum speed over all 250 ticks. Disturbance sets
`speed=0.7*currentSpeed`; recovery error is measured after 150 further ticks.
Every tick asserts finite output inside the configured voltage budget. The
identification and PI equations are in [the calculations above](#calculation-breakdown).

## Optional TalonFX adapter

### Variables to change first

| Input | Meaning |
| --- | --- |
| `motor` | The existing caller-owned TalonFX; select CAN ID/bus in your subsystem |
| `motorRotationsPerFlywheelRotation` | Positive motor turns per wheel turn; direct drive is 1 |
| `supplyCurrentLimitAmps` | Positive hardware supply-current limit; normally match the tuner config |
| Controller inversion/sign | Positive applied voltage must yield positive wheel RPS |

### How to use

Copy this optional adapter and `FlywheelAutoTuner.java` into a project with Phoenix 6.
It is not needed for simulations or non-Phoenix motors.

```java
var config = new FlywheelAutoTuner.Config(maxRPS, maxAmps, maxVolts, low, high, lambda);
var io = new FlywheelAutoTunerTalonFXIO(existingTalonFX, ratio, config.maxCurrentAmps());
var tuner = new FlywheelAutoTuner(io, config);
```

Construction enables/applies the supplied supply-current limit. It throws if
configuration fails or ratio/current is invalid. It does not create/close the
motor or select its ID, bus, inversion, or all other configuration fields.
Grant exclusive ownership while tuning and configure appropriate signal rates.

### Expected behavior and limitations

`readInputs()` reads rotor velocity, motor voltage, and supply current; its
connected flag requires all three Phoenix signal statuses to be OK.
`setVoltage` forwards the command directly. The tuner performs its own
software limits using these measurements. Status OK is not an independent
timestamp/freshness audit; confirm signal-update behavior on your CAN setup.

The original robot still selects CAN ID 13 for its tuner, which is also the top
shooter ID. This adapter deliberately leaves motor ownership to the caller;
resolve competing controller access before using that wiring on hardware.
New projects should reuse their actual motor object, not duplicate an ID.

### Calculation breakdown

For ratio `G=motorTurns/wheelTurn`, `wheelRPS=rotorRPS/G`. No conversion is
needed for reported volts or supply amperes. The adapter reports supply current,
not stator current, so its current threshold must use the supply-current units.
It calculates no tuning gains; see [the calculations above](#calculation-breakdown) for
the identification equations.

## Running and copying the simulator

Requires Java 17+, WPILib, its command framework, and release desktop JNI:
`nativeRelease wpi.java.deps.wpilibJniRelease(wpi.platforms.desktop)`.
Copy this mechanism's tuner and simulation IO/test pair, keeping its folder under
`AutoTuners` alongside the shared `tuner-simulation.gradle` and optional `run.ps1`.
The core needs no vendor library; the optional flywheel TalonFX adapter needs Phoenix 6.

From the destination project root:

```powershell
./src/main/java/frc/robot/subsystems/AutoTuners/run.ps1 -Kind Flywheel
# All three suites (default):
./src/main/java/frc/robot/subsystems/AutoTuners/run.ps1
# Direct Gradle invocation on Windows:
./gradlew.bat --init-script src/main/java/frc/robot/subsystems/AutoTuners/tuner-simulation.gradle flywheelTunerSim
```

The runner supports `-ProjectRoot`, `-Jdk` (JDK directory), and `-Offline`.
It finds the wrapper by walking upward, honors JAVA_HOME or finds a WPILib JDK,
and restores JAVA_HOME and the working directory afterward. Use a Java version
compatible with the destination GradleRIO project. First use may need network
access to cache dependencies. Missing wrapper/JDK can be resolved with those options.
Logs are saved to `build/<kind>-sim/results.txt`; check stderr too on failure.

The helper registers each available mechanism independently and compiles only its
three Java files without annotation processors, using the destination compile
dependencies. Classes go under `build/portable-tuners/<kind>/classes`. Run tasks
extract release natives into `build/jni/release`, set java.library.path, and run
the test main with the destination runtime dependencies. Missing tasks indicate
missing source files; JNI errors indicate missing desktop native dependencies.
The helper assumes a single root Java GradleRIO project and standard native paths.

Tests step physics and FPGA time at 20 ms in simulated Test mode. Expect metrics,
`RESULT PASS`, then `BUILD SUCCESSFUL`; assertions propagate a nonzero exit.
The runner/helper calculate no gains and invoke no deployment task.
Simulation checks validate the production tuner with idealized physics; they do
not validate physical robot behavior, vendor firmware, CAN timing, or battery sag.

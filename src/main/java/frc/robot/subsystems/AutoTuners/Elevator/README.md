# Elevator tuner — ElevatorAutoTuner

This folder contains `ElevatorAutoTuner.java` (the standalone WPILib tuner), `ElevatorAutoTunerIOSim.java` (physics IO), and `ElevatorTunerSimulation.java` (integration checks).

The parent folder provides the shared `run.ps1` and `tuner-simulation.gradle`.

## Variables to change first

| Variable | Units / purpose |
| --- | --- |
| `minHeightMeters`, `maxHeightMeters` | Measured lower/upper carriage heights in meters |
| `limitMarginMeters` | Positive stopping clearance before either bound |
| `maxVelocityMetersPerSec`, `maxCurrentAmps` | Safe absolute speed/current thresholds |
| `maxVolts` | Bidirectional voltage budget, greater than 0 and at most 12 V |
| `sweepAmplitudeMeters` | Identification target amplitude, at least 0.05 m |
| `bootstrapKp`, `bootstrapKd` | Conservative voltage-based position/velocity gains for collecting motion |
| `bandwidthRadPerSec` | Desired `lambda`, greater than 0 and at most 5 |
| IO gearing/drum/stage conversion and zero | Actual carriage height/speed, positive upward |
| `homed`, `lowerLimit`, `upperLimit`, `connected` | Trusted reference, actual switch states, and fresh sensor validity |
| `dashboardPrefix`, candidate target | Unique prefix (default `Elevator`), requested height in meters |

There are no default hardware settings. Match the current threshold to your
IO's current signal, and configure hardware current limits separately. Starting
clearance to each physical bound must be at least `sweepAmplitudeMeters +
2*limitMarginMeters`. Bootstrap gains require prior mechanism knowledge.

## How to use

Copy `ElevatorAutoTuner.java` with its package folder and import:

```java
import frc.robot.subsystems.AutoTuners.Elevator.ElevatorAutoTuner;

var config = new ElevatorAutoTuner.Config(minHeight, maxHeight, margin, maxSpeed, maxAmps,
    maxVolts, sweepAmplitude, bootstrapKp, bootstrapKd, lambda);
var tuner = new ElevatorAutoTuner(existingElevatorIO, config, "LiftTuner");
```

Implement `ElevatorAutoTuner.IO` with `readInputs()` returning
`new ElevatorAutoTuner.Inputs(heightMeters, velocityMetersPerSec, appliedVolts,
currentAmps, connected, homed, lowerLimit, upperLimit)` and `setVoltage(volts)`.
Use the existing controller rather than creating a second controller for the same
CAN ID. `appliedVolts` is the voltage over the previous interval. Convert motor
rotations to actual carriage travel, including drum radius, reduction, and staging.
If no physical switches exist their flags can be false, but measured software
bounds and a trusted height reference are still required.

1. Home the elevator with its normal procedure; this tuner does not home it.
2. Position it safely away from both ends and release all limit switches.
3. Pause normal elevator control and grant exclusive motor ownership. Keep the
   command scheduler running at 20 ms.
4. With speed at most 0.1 m/s, enable Test and call `beginTuning()` once.
5. Wait 12 seconds for `Candidate gains ready` and nonnull `getCandidate()`.
6. Call `runCandidate(targetHeightMeters)` for a target strictly inside each
   bound by twice the margin. Observe it in enabled Test mode.
7. Call `stop()` on completion/interruption or disable.

Separate calibration and candidate start/end commands should require the tuner
and normal elevator subsystem. Pause competing periodic controllers too.
Identification completion and `stop()` apply zero volts and do not provide
gravity holding. Use the mechanism's support/brake procedure when output stops.

## What should happen

The carriage follows a 0.25 Hz target sweep around its starting height, with
0.7 V of additional 1.1 Hz excitation and output clipped to your voltage budget.
It moves in both directions while collecting up to 120 averaged samples.
After 12 seconds it stops output and publishes candidate gains or an
`Identification failed: ...` message. It does not automatically test or save them.

`runCandidate` changes status to `Testing candidate position gains` and holds
the requested height using constant gravity/friction feedforward and PD.
Friction can leave small residual position error because kI is zero. Targets
are direct steps, not motion profiles.

Dashboard keys beneath your prefix include `TuneStatus`, `HeightMeters`,
`TargetHeightMeters`, `TuneSamples`, `CommandVolts`, `kP/kI/kD`, `kV/kA/kG/kS`, and
`ModelRMSEVolts`. Public accessors include `getCandidate()`, `getStatus()`,
`isRunning()`, and `getSampleCount()`. `stop()` reports `Stopped` and retains
valid gains; active safety abort reports `ABORTED: ...` and clears them.

## Faults, units, and limitations

Either asserted switch, loss of homing, stale/disconnected/invalid readings
reported by IO, travel inside the margin, or excessive current/speed aborts
active identification or testing. Unsafe starts are rejected. Identification
also requires 15–25 ms loop intervals. Invalid/nonfinite targets stop output.
The tuner will not automatically move away from an asserted switch.

The model assumes constant effective mass, drum radius, and reduction, with
nonnegative gravity voltage. Variable spool radius, changing stages, and an
overbalanced counterweight may need a different model. A stalled mechanism or
insufficient sweep is rejected; do not increase excitation beyond safe travel
to force a fit.

Gain units are kP V/m, kD/kV V/(m/s), kA V/(m/s²), kG/kS V; kI is zero. Export
gains manually and convert units before using them in a duty-cycle or
motor-rotation controller. No firmware settings or project constants are
automatically saved.

## Calculation breakdown

The model is `u=kV*v+kA*a+kG+kS*sign(v)`: elevator gravity is constant, with
no angle-dependent cosine. Five 20 ms intervals form each averaged row:

```text
vMean=(heightEnd-heightStart)/windowTime
aMean=(vEnd-vStart)/windowTime
signMean=Σsign(0.5*(vPrevious+vNow))*dt/windowTime
uMean=ΣappliedVolts*dt/windowTime
Xrow=[vMean,aMean,1,signMean]
beta=[kV,kA,kG,kS]
```

Solve `(XᵀX)*beta=Xᵀu` with normalized columns and pivoted elimination. Require
at least 80 averaged samples and observed height range of at least
`0.6*sweepAmplitudeMeters`. Reject tiny columns or normalized pivots below
`1e-5`. Coefficients must be finite with `kV>0`, `kA>0`, `0<=kG<maxVolts`, and
`-0.1<=kS<2 V`. Clamp a small negative kS to zero and require voltage RMSE
`sqrt(Σ(u-predictedU)²/N) <= 0.3 V`.

Gravity cancellation gives local position polynomial
`kA*s²+(kV+kD)*s+kP`. Matching it to
`kA*(s+lambda)*(s+kV/kA+lambda)` produces:

```text
kP=kV*lambda+kA*lambda²
kD=2*kA*lambda
kI=0
```

For example, `kV=6.614231`, `kA=0.138462`, `kG=1.356923`, and `lambda=3` give
`kP≈21.08885`, `kD≈0.83077`. A 0.01 m error contributes about `0.21089 V`
of proportional feedback. During candidate control:

```text
error=targetHeight-height
friction=abs(error)>0.003 m ? kS*sign(error) : 0
volts=clamp(kG+kP*error-kD*velocity+friction, -maxVolts, maxVolts)
```

The friction rule uses position-error direction to approximate static holding.
Clipping, sampling, and imperfect friction compensation affect settling and error;
the ideal pole placement is not a guarantee of physical performance.

## Physics simulation IO

### Variables to change first

| Constructor input | Meaning |
| --- | --- |
| `motor` | WPILib `DCMotor` model and motor count |
| `massKg` | Effective moving carriage/load mass |
| `gearing` | Motor turns per drum turn |
| `drumRadiusMeters` | Effective drive radius, including the assumed linear travel model |
| `minHeightMeters`, `maxHeightMeters`, `initialHeightMeters` | Physical bounds and starting height |
| `frictionVolts` | Nonnegative Coulomb-friction voltage |
| `setSensorNoise(heightStdMeters, velocityStdMetersPerSec)` | Optional finite nonnegative measurement standard deviations |

Match tuner bounds/margins to this plant. The simple model assumes constant
radius, ratio, and mass; compound reeving must be represented consistently.

### Expected behavior and helper methods

WPILib `ElevatorSim` moves a carriage under motor force and gravity, with physical
height stops. Inputs normally report connected and homed, with physical stop
contact mapped to lower/upper switch flags. This simulated homed flag does not
perform a real homing procedure. For fault tests, override the inputs.

`setVoltage` accepts finite values within ±12 V. `getCommandVolts` reports output;
`setState(height,velocity)` applies a disturbance and resets measurement noise.
`setSensorOverride(inputs)` replaces sensor readings; null restores the plant.
`setSensorNoise` enables repeatable Gaussian noise using seed 2026. Updates use
twenty substeps and an approximate Coulomb-friction model. Bad physics values,
bad noise settings, or nonpositive/nonfinite update dt throw exceptions.

No vendor library, battery sag, cable stretch, changing spool radius, or CAN
transport is modeled. Simulated current is the effective-input plant current,
so the friction modification is not a complete electrical-loss model.

### Calculation breakdown

For mass `m`, reduction `G`, radius `r`, and motor resistance/torque/velocity
constants `R`, `Kt`, `Kv`, ideal voltage coefficients are:

```text
kV=G/(r*Kv)
kA=m*R*r/(G*Kt)
kG=9.8*kA
```

At each substep, friction direction is `sign(velocity)` above 0.002 m/s;
otherwise it is `sign(volts-kG)`. The gravity-enabled plant receives
`volts-frictionVolts*direction` for `dt/20` seconds. Independent Gaussian
height/speed noise is added to reported measurements after updating physics.

## Simulation checks

### Variables to change first

| Fixture value | Where / default |
| --- | --- |
| Physical cases | `cases`: mass, gearing, drum radius, friction, height noise, speed noise |
| Tuner configuration | `config()`: 0–1.5 m, 0.05 m margin, 1.5 m/s, 80 A, 6 V, 0.2 m sweep, bootstrap 30/2, bandwidth 3 |
| Start / targets | 0.75 m; 0.95, 0.40, 1.15 m |
| Durations | 601 identification ticks, 250 ticks per target, 150 recovery ticks |
| Disturbance / tolerance | Move carriage to 1.05 m from near 1.15 m; error under 10 mm |
| Model tolerances | kV/kG within 15%, kA within 25%, kS within 0.06 V |

These numbers are test fixtures and do not establish safe hardware limits.
The last case adds 0.5 mm height and 5 mm/s speed measurement noise.

### Expected output and coverage

Five setups print `IDENTIFY`, `POSITION`, and `RECOVERY`, followed by stalled
motion rejection and `RESULT PASS checks=7847 cases=5 plus stalled elevator`.
Each moving case normally produces 120 samples. Current worst final-second
height error is about 3.98 mm with friction/noise; worst recovery error is about
2.96 mm. Assertions use a 10 mm limit, not those exact repeatable values.

Tests cover mode/candidate requirements, invalid targets, voltage limits,
disabled output, active switches/travel/speed/current faults, lost homing or
connection, invalid sensor values, unsafe restart, movement at start,
insufficient clearance, bad identification timing, and stalled-motion rejection.
They do not establish real overshoot, cable behavior, battery sag, or CAN latency.
No wrapper, arm/flywheel simulator, or motor-vendor library is required.

### Calculation breakdown

Independent expected values are `kV=G/(r*Kv)`, `kA=m*R*r/(G*Kt)`, and
`kG=9.8*kA`. The test compares relative model errors against configured
tolerances and absolute kS error against 0.06 V. Each target's metric is
`1000*max(abs(measuredHeight-target))` across its final 50 ticks. Recovery error
is `1000*abs(measuredHeight-1.15)` after a 0.10 m downward disturbance and 150
ticks. See [the calculations above](#calculation-breakdown) for the fitted model and PD
gain derivation.

## Running and copying the simulator

Requires Java 17+, WPILib, its command framework, and release desktop JNI:
`nativeRelease wpi.java.deps.wpilibJniRelease(wpi.platforms.desktop)`.
Copy this mechanism's tuner and simulation IO/test pair, keeping its folder under
`AutoTuners` alongside the shared `tuner-simulation.gradle` and optional `run.ps1`.
The core needs no vendor library; the optional flywheel TalonFX adapter needs Phoenix 6.

From the destination project root:

```powershell
./src/main/java/frc/robot/subsystems/AutoTuners/run.ps1 -Kind Elevator
# All three suites (default):
./src/main/java/frc/robot/subsystems/AutoTuners/run.ps1
# Direct Gradle invocation on Windows:
./gradlew.bat --init-script src/main/java/frc/robot/subsystems/AutoTuners/tuner-simulation.gradle elevatorTunerSim
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

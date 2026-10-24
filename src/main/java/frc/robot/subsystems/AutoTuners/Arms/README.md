# Arm tuner — ArmAutoTuner

This folder contains `ArmAutoTuner.java` (the standalone WPILib tuner), `ArmAutoTunerIOSim.java` (physics IO), and `ArmTunerSimulation.java` (integration checks).

The parent folder provides the shared `run.ps1` and `tuner-simulation.gradle`.

## Variables to change first

| Variable | Units / purpose |
| --- | --- |
| `minAngleRad`, `maxAngleRad` | Physical mechanism angle bounds, radians from horizontal |
| `limitMarginRad` | Positive clearance before either bound |
| `maxVelocityRadPerSec`, `maxCurrentAmps` | Safe absolute speed/current thresholds |
| `maxVolts` | Bidirectional voltage budget, greater than 0 and at most 12 V |
| `sweepAmplitudeRad` | Identification target amplitude, at least 0.15 rad |
| `bootstrapKp`, `bootstrapKd` | Conservative voltage-based position/velocity gains used to collect data |
| `bandwidthRadPerSec` | Desired `lambda`, greater than 0 and at most 5 |
| IO encoder zero/gearing/polarity | Zero at horizontal; positive voltage increases angle |
| `dashboardPrefix`, test angle | Optional unique prefix (default `Arm`), requested angle in radians |

There are no hardware defaults. Starting clearance to each bound must be at
least `sweepAmplitudeRad + 2*limitMarginRad`. Verify bootstrap gains for your
mechanism before starting; this routine cannot identify an unknown arm safely
without a valid reference, reasonable initial control, and safe limits.

## How to use

Copy `ArmAutoTuner.java` with its package folder and import:

```java
import frc.robot.subsystems.AutoTuners.Arms.ArmAutoTuner;

var config = new ArmAutoTuner.Config(minAngle, maxAngle, margin, maxSpeed, maxAmps,
    maxVolts, sweepAmplitude, bootstrapKp, bootstrapKd, lambda);
var tuner = new ArmAutoTuner(existingArmIO, config, "PivotTuner");
```

`ArmAutoTuner.IO` reads `new ArmAutoTuner.Inputs(angleRad, velocityRadPerSec, appliedVolts,
currentAmps, connected)` and commands voltage through the existing motor.
Convert gearing and encoder offset before returning values. If an encoder has
motor turns `r` and reduction `G`, mechanism angle is `2*pi*r/G + horizontalOffset`;
motor RPM converts to rad/s as `2*pi*RPM/(60*G)`. Account for any conversion
already configured on the encoder, so it is not applied twice.

1. Establish a known horizontal angle reference and select a safe starting angle.
2. Pause normal arm control; give the tuner exclusive motor ownership and run
   the command scheduler at 20 ms.
3. With absolute velocity at most 0.1 rad/s, enable Test and call `beginTuning()`.
4. Let collection run for 12 seconds. Call it only once, not every periodic loop.
5. After `Candidate gains ready` and nonnull `getCandidate()`, call
   `runCandidate(targetAngleRad)` for a target strictly inside both bounds by
   twice the margin. Keep Test enabled.
6. Observe angle/current/output; call `stop()` when finished or interrupted.

Calibration and testing should be separate start/end commands, requiring both
the tuner and existing arm subsystem. Pause any conflicting controller in the
arm subsystem's `periodic()` too. Identification completion and `stop()` command
zero volts; support/brake the arm according to your mechanism's procedure.

## What should happen

The bootstrap controller follows a 0.25 Hz target sweep around the start angle
with 0.7 V of additional 1.1 Hz excitation, clipped to your voltage budget. This
produces upward/downward motion for gravity/friction identification. It gathers
up to 120 averaged samples over 12 seconds.

At completion output becomes zero and the tuner either publishes gains or
reports `Identification failed: ...`. Testing a candidate then holds the selected
angle with gravity/friction feedforward and PD feedback. The arm does not
automatically move to a candidate target or save gains. Small friction-related
position error can remain because kI is zero.

Dashboard keys beneath your prefix include `TuneStatus`, `AngleRad`,
`TargetAngleRad`, `TuneSamples`, `CommandVolts`, `kP/kI/kD`, `kV/kA/kG/kS`, and
`ModelRMSEVolts`. `getCandidate()`, `getStatus()`, and `isRunning()` are available
programmatically. `stop()` reports `Stopped` and retains a previously valid
candidate; active safety aborts report `ABORTED: ...` and clear it.

## Limits, interpretation, and troubleshooting

Travel inside the configured margin, excessive absolute speed/current, bad
sensor values, or lost connection aborts active tuning/testing. Identification
also aborts if loop intervals leave 15–25 ms. Invalid targets stop output. The
arm core has no physical switch or homing inputs: include those conditions in
the IO's `connected` validity flag if needed. Your IO must identify stale data.

The fit needs at least 80 averaged samples and at least 0.25 rad of observed
travel. If movement is insufficient, check geometry, voltage limits, and
bootstrap control. Poor fit can mean incorrect angle zero/sign, friction,
backlash, sensor timing, or an unsuitable model. Do not increase movement into
unsafe travel to force a fit. If the mechanism differs substantially from a
gravity-loaded rigid arm, this model may not apply.

Use fitted gains in voltage/radian units: kP V/rad, kD and kV V/(rad/s),
kA V/(rad/s²), kG/kS V; kI is zero. Gains need conversion before use in a
duty-cycle or rotation-based controller. They are volatile and do not update
motor firmware automatically. Motion targets are unprofiled steps; use a
profiled normal controller if required by the mechanism.

## Calculation breakdown

The model is `u=kV*v+kA*a+kG*cos(theta)+kS*sign(v)`.
Every five 20 ms readings form an approximately 0.1 s regression window:

```text
vMean=(thetaEnd-thetaStart)/windowTime
aMean=(vEnd-vStart)/windowTime
cosMean=Σ[0.5*(cos(thetaPrevious)+cos(thetaNow))*dt]/windowTime
signMean=Σsign(0.5*(vPrevious+vNow))*dt/windowTime
uMean=ΣappliedVolts*dt/windowTime
```

Each row of `X` is `[vMean,aMean,cosMean,signMean]`; fit
`beta=[kV,kA,kG,kS]` from `X*beta≈u`. The code builds `XᵀX` and `Xᵀu`, normalizes
columns by `sqrt(ΣXcolumn²)`, and solves with pivoted elimination. It rejects
tiny/poorly excited columns or normalized pivots below `1e-5`.

Coefficients must be finite with `kV>0`, `kA>0`, `0<=kG<maxVolts`, and
`-0.1<=kS<2 V`. A small negative kS is clamped to zero, then voltage RMSE is
computed and must be at most 0.3 V. These are fit checks on the collected data,
not independent proof of hardware performance.

With perfect gravity compensation, local position dynamics give
`kA*s²+(kV+kD)*s+kP`. Choosing poles at `-lambda` and `-(kV/kA+lambda)` yields:

```text
kP=kV*lambda+kA*lambda²
kD=2*kA*lambda
kI=0
```

For example, `kV=0.198423`, `kA=0.092312`, and `lambda=3` give
`kP≈1.42608`, `kD≈0.55387`. During candidate control:

```text
error=targetAngle-angle
friction=abs(error)>0.02 rad ? kS*sign(error) : 0
volts=clamp(kG*cos(angle)+kP*error-kD*velocity+friction, -maxVolts, maxVolts)
```

The position-error friction rule approximates static compensation; it is not
the velocity-sign identification model evaluated at a trajectory. Imperfect
friction/gravity compensation, clipping, and sampling can leave error or overshoot.

## Physics simulation IO

### Variables to change first

| Input | Meaning |
| --- | --- |
| `motor` | Optional explicit WPILib `DCMotor`; overload without it uses one NEO |
| `inertia`, `gearing`, `length` | Mechanism kg·m², motor turns/arm turn, arm length in meters |
| `minAngle`, `maxAngle`, `initialAngle` | Physical bounds and start, radians from horizontal |
| `frictionVolts` | Nonnegative Coulomb-friction voltage |
| `setSensorNoise(angleStdRad, velocityStdRadPerSec)` | Optional measurement noise standard deviations; use finite nonnegative values |

Match the tuner's software bounds to the physical plant with appropriate margins.
Values from the test main are illustrative fixtures, not measured robot settings.

### Expected behavior and helper methods

The arm accelerates under motor torque and gravity with physical angular stops.
`setVoltage()` accepts finite output within ±12 V. `readInputs()` reports angle,
speed, commanded volts, simulated motor current, and valid connectivity.
`setState(angle, velocity)` applies a physical disturbance and clears existing
noise. `setSensorOverride(inputs)` injects sensor faults; null restores physics.
Noise uses fixed random seed 2026 for repeatable runs.

Each update uses twenty physics substeps. Optional friction opposes motion;
near zero velocity its direction is chosen using commanded voltage minus the
estimated gravity voltage. This is an approximate friction model rather than
a complete static-friction solver. Current is the underlying effective-input
plant current, so friction-related electrical behavior is approximate too.
No flexible links, backlash, battery sag, or real sensor transport are modeled.

### Calculation breakdown

The gravity-enabled `SingleJointedArmSim` uses the uniform-arm relation
`mass=3*J/length²`. For reduction `G`, motor resistance `R`, torque constant `Kt`,
and velocity constant `Kv`:

```text
kV=G/Kv
kA=J*R/(G*Kt)
kG=(mass*9.8*length/2)*R/(G*Kt)
gravityVoltageAtAngle=kG*cos(angle)
```

Within each update, `dtSub=dt/20`. Above 0.005 rad/s, friction direction is
`sign(velocity)`; otherwise it is `sign(volts-kG*cos(angle))`. Physics receives
`volts-frictionVolts*direction`. Measurement noise adds independent Gaussian
samples scaled by the configured standard deviations after the physical update.

## Simulation checks

### Variables to change first

| Fixture value | Where / default |
| --- | --- |
| Physical cases | `cases`: inertia, reduction, length, friction, angle noise, speed noise |
| Tuner configuration | `config()`: -1.1 to 1.5 rad, 0.08 rad margin, 4 rad/s, 60 A, 6 V, 0.4 rad sweep, bootstrap 12/1, bandwidth 3 |
| Initial angle / targets | 0.2 rad; 0.7, -0.3, 0.95 rad |
| Durations | 601 identification ticks, 250 ticks per target, 150 recovery ticks |
| Disturbance / tolerance | Move to 0.8 rad; tracking/recovery error under 2° |
| Model tolerances | kV/kG within 15%, kA within 25%, kS within 0.06 V |

These are simulated fixtures, not recommended hardware settings. The physics
assumes the WPILib uniform-arm gravity model.

### Expected output and coverage

Five physical/noise cases print `IDENTIFY`, three `POSITION` results, and
`RECOVERY`; a separate immobile IO prints `STALLED`. Completion is
`RESULT PASS checks=7782 cases=5 plus stalled arm`. Current worst final-second
error is about 1.33° with friction/noise, and worst recovery error about 1.15°.
A failure throws an assertion and exits nonzero.

Cases include 10:1, 30:1, and 60:1 reductions, friction, deterministic noise,
and stalled motion. Additional checks cover mode/candidate requirements,
target validation, voltage bounds, disabling, active travel/speed/current/sensor
faults, insufficient sweep clearance, a moving start, and bad identification
timing. It tests the production algorithm, not a second implementation.

### Calculation breakdown

Independent expected coefficients are `kV=G/Kv`, `kA=J*R/(G*Kt)`, and
`kG=(3*J/length²)*9.8*length/2*R/(G*Kt)` for the WPILib uniform arm. Relative
coefficient error is `abs(fitted/expected-1)`; friction error is absolute volts.
For each five-second target run, take the maximum absolute angle error in its
final 50 ticks and convert radians to degrees with `180/pi`. Recovery compares
the final angle with 0.95 rad after three seconds. All must remain below 2°.
See [the calculations above](#calculation-breakdown) for regression and controller derivations.

## Running and copying the simulator

Requires Java 17+, WPILib, its command framework, and release desktop JNI:
`nativeRelease wpi.java.deps.wpilibJniRelease(wpi.platforms.desktop)`.
Copy this mechanism's tuner and simulation IO/test pair, keeping its folder under
`AutoTuners` alongside the shared `tuner-simulation.gradle` and optional `run.ps1`.
The core needs no vendor library; the optional flywheel TalonFX adapter needs Phoenix 6.

From the destination project root:

```powershell
./src/main/java/frc/robot/subsystems/AutoTuners/run.ps1 -Kind Arm
# All three suites (default):
./src/main/java/frc/robot/subsystems/AutoTuners/run.ps1
# Direct Gradle invocation on Windows:
./gradlew.bat --init-script src/main/java/frc/robot/subsystems/AutoTuners/tuner-simulation.gradle armTunerSim
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

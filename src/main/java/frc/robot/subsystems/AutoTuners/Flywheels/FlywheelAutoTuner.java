package frc.robot.subsystems.AutoTuners.Flywheels;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Portable Test-mode flywheel tuner. Caller owns the motor and its exclusive use.
 * Uses only WPILib; implement IO or copy the optional vendor-specific adapter.
 */
public final class FlywheelAutoTuner extends SubsystemBase {
    /** Mechanism velocity is flywheel rotations/second, after gearing conversion. */
    public record Inputs(double velocityRPS, double appliedVolts, double currentAmps, boolean connected) {}
    public interface IO {
        /** Fresh readings; appliedVolts describes the preceding control interval. */
        Inputs readInputs();
        void setVoltage(double volts);
    }
    /** Configure safe mechanism limits before testing; current units must match the IO. */
    public record Config(double maxTestRPS, double maxCurrentAmps, double maxRunVolts,
                         double lowTestVolts, double highTestVolts, double bandwidthRadPerSec) {
        public Config {
            for (double value : new double[] {maxTestRPS, maxCurrentAmps, maxRunVolts,
                    lowTestVolts, highTestVolts, bandwidthRadPerSec})
                if (!Double.isFinite(value)) throw new IllegalArgumentException("Configuration must be finite");
            if (!(maxTestRPS > 0 && maxCurrentAmps > 0 && maxRunVolts > 1 && maxRunVolts <= 12
                    && lowTestVolts > 0 && highTestVolts > lowTestVolts
                    && highTestVolts <= maxRunVolts && bandwidthRadPerSec > 0 && bandwidthRadPerSec <= 10))
                throw new IllegalArgumentException("Invalid flywheel tuning configuration");
        }
        /** Example limits, not measured limits for a new mechanism. */
        public static Config defaults() { return new Config(75, 35, 8, 2, 4, 4); }
    }
    private static final double DT = 0.020; // Time intervals
    private final IO io;
    private final Config config;
    private final String dashboardPrefix;
    private String tuneStatus = "Idle";

    private final PIDController pid =
            new PIDController(0, 0, 0, DT);

    private enum Mode {
        OFF, IDENTIFY, RUN
    }

    private Mode mode = Mode.OFF;

    private record Sample(
            double previousVelocity,
            double voltage,
            double nextVelocity
    ) {}

    public record Gains(
            double kP,
            double kI,
            double kD,
            double kV,
            double a,
            double b,
            double rmse
    ) {}

    private final List<Sample> samples =
            new ArrayList<>();

    private Gains candidate = null;

    private double startTime;
    private double previousTime = Double.NaN;
    private double previousVelocity;
    private double previousCommand;
    private double maxObservedVelocity;

    private double targetRPS = 0;

    public FlywheelAutoTuner(IO io, Config config) { this(io, config, "Flywheel"); }
    public FlywheelAutoTuner(IO io, Config config, String dashboardPrefix) {
        this.io = Objects.requireNonNull(io);
        this.config = Objects.requireNonNull(config);
        if (dashboardPrefix == null || dashboardPrefix.isBlank())
            throw new IllegalArgumentException("Dashboard prefix is required");
        this.dashboardPrefix = dashboardPrefix;
        pid.setIntegratorRange(-2.0, 2.0);
        status("Idle");
    }
    public Gains getCandidate() { return candidate; }
    public String getStatus() { return tuneStatus; }
    public boolean isRunning() { return mode == Mode.RUN; }
    public int getSampleCount() { return samples.size(); }

    private void status(String message) {
        tuneStatus = message;
        SmartDashboard.putString(
                dashboardPrefix + "/TuneStatus", message
        );
    }

    // ------------------------
    // START CALIBRATION
    // ------------------------

    public void beginTuning() {
        if (!DriverStation.isTestEnabled()) {
            stop();
            status("Calibration requires Test mode");
            return;
        }

        Inputs in = io.readInputs();
        if (!safe(in)) { abort("Velocity or current safety limit"); return; }
        if (Math.abs(in.velocityRPS()) > 2.0) {
            stop();
            status("Flywheel must be stopped first");
            return;
        }

        io.setVoltage(0);
        samples.clear();
        candidate = null;

        maxObservedVelocity = 0;
        previousTime = Double.NaN;
        previousCommand = 0;

        startTime = Timer.getFPGATimestamp();
        mode = Mode.IDENTIFY;

        status("Collecting motor data");
    }

    // Voltage sequence: 0 -> configured low -> configured high -> low -> 0.
    private double testVoltage(double elapsed) {
        if (elapsed < 0.5) return 0.0;
        if (elapsed < 2.0) return config.lowTestVolts();
        if (elapsed < 3.5) return config.highTestVolts();
        if (elapsed < 5.0) return config.lowTestVolts();
        return 0.0;
    }

    private void abort(String reason) {
        io.setVoltage(0);
        candidate = null;
        mode = Mode.OFF;
        status("ABORTED: " + reason);
    }

    public void stop() {
        io.setVoltage(0);
        mode = Mode.OFF;
        pid.reset();
    }

    // ------------------------
    // SYSTEM IDENTIFICATION
    // ------------------------

    private Gains calculateGains() {
        if (samples.size() < 80) {
            throw new IllegalStateException(
                    "Not enough valid samples"
            );
        }

        // Least squares:
        // w[k+1] = A*w[k] + B*u[k]

        double sVV = 0;
        double sVU = 0;
        double sUU = 0;
        double sVY = 0;
        double sUY = 0;

        for (Sample s : samples) {
            double v = s.previousVelocity();
            double u = s.voltage();
            double y = s.nextVelocity();

            sVV += v * v;
            sVU += v * u;
            sUU += u * u;
            sVY += v * y;
            sUY += u * y;
        }

        double det = sVV * sUU - sVU * sVU;

        if (det <= 1e-4 * sVV * sUU) {
            throw new IllegalStateException(
                    "Insufficient excitation"
            );
        }

        double A =
                (sVY * sUU - sUY * sVU) / det;

        double B =
                (sUY * sVV - sVY * sVU) / det;

        if (!(A > 0 && A < 1 && B > 0)) {
            throw new IllegalStateException(
                    "Invalid identified model"
            );
        }

        // Convert discrete coefficients
        // to continuous-time coefficients.

        double a = -Math.log(A) / DT;
        double b = B * a / (1.0 - A);

        if (!Double.isFinite(a) ||
                !Double.isFinite(b) ||
                b <= 0) {
            throw new IllegalStateException(
                    "Invalid model parameters"
            );
        }

        // Check how well the model predicts
        // the measured velocity.

        double squaredError = 0;

        for (Sample s : samples) {
            double prediction =
                    A * s.previousVelocity()
                            + B * s.voltage();

            double error =
                    s.nextVelocity() - prediction;

            squaredError += error * error;
        }

        double rmse = Math.sqrt(
                squaredError / samples.size()
        );

        if (rmse > Math.max(
                0.30, maxObservedVelocity * 0.03)) {
            throw new IllegalStateException(
                    "Poor model fit"
            );
        }

        if (maxObservedVelocity < 5.0) {
            throw new IllegalStateException(
                    "Flywheel did not accelerate enough"
            );
        }

        // Pole-placement PI design
        double lambda = config.bandwidthRadPerSec();
        double kP = (2.0 * lambda - a) / b;
        double kI = lambda * lambda / b;

        // First-order steady-state feedforward
        double kV = a / b;

        // The initial design uses no derivative.
        double kD = 0.0;

        if (!(Double.isFinite(kP) && Double.isFinite(kI) && Double.isFinite(kV)
                && kP > 0 && kI > 0 && kV > 0)) {
            throw new IllegalStateException(
                    "Invalid gains; check model and configured bandwidth"
            );
        }

        return new Gains(
                kP, kI, kD, kV, a, b, rmse
        );
    }

    // ------------------------
    // ACTIVATE CANDIDATE
    // ------------------------

    public void runCandidate(double requestedRPS) {
        if (!DriverStation.isTestEnabled()
                || candidate == null
                || mode == Mode.IDENTIFY) {
            stop();
            status("No valid candidate or not in Test mode");
            return;
        }

        if (!Double.isFinite(requestedRPS) || requestedRPS <= 0 ||
                requestedRPS > config.maxTestRPS()) {
            stop();
            status("Invalid test speed");
            return;
        }

        if (candidate.kV() * requestedRPS
                > config.maxRunVolts() - 1.0) {
            stop();
            status("Feedforward exceeds test voltage budget");
            return;
        }

        if (!safe(io.readInputs())) { abort("Velocity or current safety limit"); return; }

        pid.setPID(
                candidate.kP(),
                candidate.kI(),
                candidate.kD()
        );

        pid.reset();
        targetRPS = requestedRPS;
        mode = Mode.RUN;

        status("Testing candidate PI gains");
    }

    // ------------------------
    // PERIODIC LOOP
    // ------------------------

    @Override
    public void periodic() {
        Inputs in = io.readInputs();
        double velocity = in == null ? Double.NaN : in.velocityRPS();

        SmartDashboard.putNumber(
                dashboardPrefix + "/VelocityRPS", velocity
        );

        if (!DriverStation.isTestEnabled()) {
            if (mode != Mode.OFF) stop();
            return;
        }

        if (mode == Mode.OFF) return;

        if (!safe(in)) {
            abort("Velocity or current safety limit");
            return;
        }

        if (mode == Mode.RUN) {
            double ff = candidate.kV() * targetRPS;

            double feedback = pid.calculate(
                    velocity, targetRPS
            );

            double volts = MathUtil.clamp(
                    ff + feedback,
                    0.0,
                    config.maxRunVolts()
            );

            io.setVoltage(volts);

            SmartDashboard.putNumber(
                    dashboardPrefix + "/TargetRPS", targetRPS
            );

            SmartDashboard.putNumber(
                    dashboardPrefix + "/CommandVolts", volts
            );

            return;
        }

        if (mode != Mode.IDENTIFY) return;

        double now = Timer.getFPGATimestamp();
        double elapsed = now - startTime;

        if (elapsed >= 5.0) {
            io.setVoltage(0);
            mode = Mode.OFF;

            try {
                candidate = calculateGains();

                SmartDashboard.putNumber(
                        dashboardPrefix + "/kP", candidate.kP()
                );
                SmartDashboard.putNumber(
                        dashboardPrefix + "/kI", candidate.kI()
                );
                SmartDashboard.putNumber(
                        dashboardPrefix + "/kD", candidate.kD()
                );
                SmartDashboard.putNumber(
                        dashboardPrefix + "/kV", candidate.kV()
                );
                SmartDashboard.putNumber(
                        dashboardPrefix + "/a", candidate.a()
                );
                SmartDashboard.putNumber(
                        dashboardPrefix + "/b", candidate.b()
                );
                SmartDashboard.putNumber(
                        dashboardPrefix + "/ModelRMSE",
                        candidate.rmse()
                );

                status("Candidate gains ready");

            } catch (IllegalStateException ex) {
                candidate = null;
                status("Identification failed: "
                        + ex.getMessage());
            }

            return;
        }

        double command = testVoltage(elapsed);

        maxObservedVelocity = Math.max(
                maxObservedVelocity,
                Math.abs(velocity)
        );

        double actualVolts = in.appliedVolts();

        // Only collect pairs within a stable
        // voltage segment, near the 20 ms period.
        if (Double.isFinite(previousTime)) {
            double dt = now - previousTime;

            boolean validTiming =
                    dt >= 0.015 && dt <= 0.025;

            boolean unchangedCommand =
                    Math.abs(command - previousCommand)
                            < 0.001;

            boolean voltageTracking =
                    Math.abs(actualVolts - previousCommand)
                            < 0.4;

            if (validTiming &&
                    unchangedCommand &&
                    voltageTracking &&
                    command > 0) {

                samples.add(new Sample(
                        previousVelocity,
                        actualVolts,
                        velocity
                ));
            }
        }

        io.setVoltage(command);

        previousTime = now;
        previousVelocity = velocity;
        previousCommand = command;

        SmartDashboard.putNumber(
                dashboardPrefix + "/TuneSamples", samples.size()
        );
    }

    private boolean safe(Inputs in) {
        return in != null && in.connected() && Double.isFinite(in.velocityRPS())
            && Double.isFinite(in.appliedVolts()) && Double.isFinite(in.currentAmps())
            && Math.abs(in.velocityRPS()) <= config.maxTestRPS()
            && Math.abs(in.currentAmps()) <= config.maxCurrentAmps() + 2;
    }
}

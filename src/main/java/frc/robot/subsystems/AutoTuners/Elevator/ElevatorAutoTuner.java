package frc.robot.subsystems.AutoTuners.Elevator;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Portable Test-mode elevator identification and position control in meters.
 * Positive voltage must lift the carriage. Caller owns the motor, establishes a homed height,
 * configures hardware current limits, and selects safe Config values. No vendor library required.
 */
public final class ElevatorAutoTuner extends SubsystemBase {
    public record Inputs(double heightMeters, double velocityMetersPerSec, double appliedVolts,
                         double currentAmps, boolean connected, boolean homed,
                         boolean lowerLimit, boolean upperLimit) {}
    public interface IO {
        /** Fresh, synchronized mechanism readings; connected must include sensor validity.
         * appliedVolts is the voltage held over the preceding control interval.
         */
        Inputs readInputs();
        void setVoltage(double volts);
    }
    /** Bootstrap gains are conservative position/velocity gains used only to collect data.
     * They must be checked for the mechanism before enabling identification.
     */
    public record Config(double minHeightMeters, double maxHeightMeters, double limitMarginMeters,
                         double maxVelocityMetersPerSec, double maxCurrentAmps, double maxVolts,
                         double sweepAmplitudeMeters, double bootstrapKp, double bootstrapKd,
                         double bandwidthRadPerSec) {
        public Config {
            double[] values = {minHeightMeters, maxHeightMeters, limitMarginMeters, maxVelocityMetersPerSec,
                maxCurrentAmps, maxVolts, sweepAmplitudeMeters, bootstrapKp, bootstrapKd, bandwidthRadPerSec};
            for (double value : values) if (!Double.isFinite(value))
                throw new IllegalArgumentException("Configuration must be finite");
            if (!(minHeightMeters < maxHeightMeters && limitMarginMeters > 0
                    && 2 * limitMarginMeters < maxHeightMeters - minHeightMeters
                    && maxVelocityMetersPerSec > 0 && maxCurrentAmps > 0
                    && maxVolts > 0 && maxVolts <= 12 && sweepAmplitudeMeters >= 0.05
                    && bootstrapKp > 0 && bootstrapKd >= 0
                    && bandwidthRadPerSec > 0 && bandwidthRadPerSec <= 5))
                throw new IllegalArgumentException("Invalid elevator tuning configuration");
        }
    }
    /** Voltage units: kP V/m, kD and kV V/(m/s), kA V/(m/s²).
     * kG and kS are volts. kI is zero: gravity is handled by feedforward.
     */
    public record Gains(double kP, double kI, double kD, double kV, double kA,
                        double kG, double kS, double rmseVolts) {}
    private enum Mode { OFF, IDENTIFY, RUN }
    private record Sample(double v, double acceleration, double direction, double volts) {}
    private final IO io;
    private final Config config;
    private final String dashboardPrefix;
    private final List<Sample> samples = new ArrayList<>();
    private Mode mode = Mode.OFF;
    private Gains candidate;
    private String status = "Idle";
    private Inputs previous;
    private double previousTime, startTime, startHeight, targetHeight;
    private double minObserved, maxObserved;
    private double windowTime, integralV, integralSign, integralVolts, initialVelocity;
    private int windowTicks;

    public ElevatorAutoTuner(IO io, Config config) {
        this(io, config, "Elevator");
    }
    public ElevatorAutoTuner(IO io, Config config, String dashboardPrefix) {
        this.io = Objects.requireNonNull(io);
        this.config = Objects.requireNonNull(config);
        if (dashboardPrefix == null || dashboardPrefix.isBlank())
            throw new IllegalArgumentException("Dashboard prefix is required");
        this.dashboardPrefix = dashboardPrefix;
        publishStatus("Idle");
    }
    public Gains getCandidate() { return candidate; }
    public String getStatus() { return status; }
    public boolean isRunning() { return mode == Mode.RUN; }
    public int getSampleCount() { return samples.size(); }
    private void publishStatus(String message) {
        status = message;
        SmartDashboard.putString(dashboardPrefix + "/TuneStatus", message);
    }
    private boolean safe(Inputs in) {
        return in != null && in.connected() && in.homed() && !in.lowerLimit() && !in.upperLimit()
            && Double.isFinite(in.heightMeters())
            && Double.isFinite(in.velocityMetersPerSec()) && Double.isFinite(in.appliedVolts())
            && Double.isFinite(in.currentAmps()) && Math.abs(in.currentAmps()) <= config.maxCurrentAmps()
            && Math.abs(in.velocityMetersPerSec()) <= config.maxVelocityMetersPerSec()
            && in.heightMeters() > config.minHeightMeters() + config.limitMarginMeters()
            && in.heightMeters() < config.maxHeightMeters() - config.limitMarginMeters();
    }
    public void stop() {
        io.setVoltage(0);
        mode = Mode.OFF;
        publishStatus("Stopped");
    }
    private void abort(String reason) {
        stop();
        candidate = null;
        publishStatus("ABORTED: " + reason);
    }
    public void beginTuning() {
        if (!DriverStation.isTestEnabled()) { stop(); publishStatus("Calibration requires Test mode"); return; }
        Inputs in = io.readInputs();
        if (!safe(in)) { abort("Unsafe or invalid elevator sensors"); return; }
        if (Math.abs(in.velocityMetersPerSec()) > 0.1) { stop(); publishStatus("Elevator must be stopped first"); return; }
        double clearance = Math.min(in.heightMeters() - config.minHeightMeters(), config.maxHeightMeters() - in.heightMeters());
        if (clearance < config.sweepAmplitudeMeters() + 2 * config.limitMarginMeters()) {
            stop(); publishStatus("Not enough travel for identification sweep"); return;
        }
        io.setVoltage(0);
        samples.clear();
        candidate = null;
        previous = in;
        startHeight = in.heightMeters();
        minObserved = maxObserved = startHeight;
        startTime = previousTime = Timer.getFPGATimestamp();
        resetWindow(in.velocityMetersPerSec());
        mode = Mode.IDENTIFY;
        publishStatus("Collecting elevator data");
    }
    public void runCandidate(double heightMeters) {
        if (!Double.isFinite(heightMeters) || heightMeters <= config.minHeightMeters() + 2 * config.limitMarginMeters()
                || heightMeters >= config.maxHeightMeters() - 2 * config.limitMarginMeters()) {
            stop(); publishStatus("Invalid test height"); return;
        }
        if (!DriverStation.isTestEnabled() || candidate == null || mode == Mode.IDENTIFY) {
            stop(); publishStatus("No valid candidate or not in Test mode"); return;
        }
        if (!safe(io.readInputs())) { abort("Unsafe or invalid elevator sensors"); return; }
        targetHeight = heightMeters;
        mode = Mode.RUN;
        publishStatus("Testing candidate position gains");
    }
    private void resetWindow(double velocity) {
        windowTicks = 0;
        windowTime = integralV = integralSign = integralVolts = 0;
        initialVelocity = velocity;
    }
    private void collect(Inputs in, double dt) {
        // Integrate over 100 ms instead of differentiating noisy individual velocity readings.
        double v = (previous.velocityMetersPerSec() + in.velocityMetersPerSec()) / 2;
        integralV += in.heightMeters() - previous.heightMeters();
        integralSign += Math.signum(v) * dt;
        // readInputs reports voltage actually applied during the interval that just elapsed.
        integralVolts += in.appliedVolts() * dt;
        windowTime += dt;
        if (++windowTicks == 5) {
            samples.add(new Sample(integralV / windowTime,
                (in.velocityMetersPerSec() - initialVelocity) / windowTime,
                integralSign / windowTime, integralVolts / windowTime));
            resetWindow(in.velocityMetersPerSec());
        }
    }
    private Gains calculateGains() {
        if (samples.size() < 80 || maxObserved - minObserved < config.sweepAmplitudeMeters() * 0.6)
            throw new IllegalStateException("Not enough elevator motion or valid samples");
        // u = kV*velocity + kA*acceleration + kG + kS*sign(velocity).
        double[][] normal = new double[4][5];
        for (Sample s : samples) {
            double[] x = {s.v(), s.acceleration(), 1.0, s.direction()};
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) normal[i][j] += x[i] * x[j];
                normal[i][4] += x[i] * s.volts();
            }
        }
        // Normalize columns so excitation checks do not depend on coefficient units.
        double[] scale = new double[4];
        for (int i = 0; i < 4; i++) {
            scale[i] = Math.sqrt(normal[i][i]);
            if (!(scale[i] > 1e-6)) throw new IllegalStateException("Insufficient excitation");
        }
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) normal[i][j] /= scale[i] * scale[j];
            normal[i][4] /= scale[i];
        }
        for (int col = 0; col < 4; col++) {
            int pivot = col;
            for (int row = col + 1; row < 4; row++)
                if (Math.abs(normal[row][col]) > Math.abs(normal[pivot][col])) pivot = row;
            double[] swap = normal[col]; normal[col] = normal[pivot]; normal[pivot] = swap;
            if (Math.abs(normal[col][col]) < 1e-5) throw new IllegalStateException("Insufficient excitation");
            double divisor = normal[col][col];
            for (int j = col; j <= 4; j++) normal[col][j] /= divisor;
            for (int row = 0; row < 4; row++) if (row != col) {
                double factor = normal[row][col];
                for (int j = col; j <= 4; j++) normal[row][j] -= factor * normal[col][j];
            }
        }
        double kV = normal[0][4] / scale[0], kA = normal[1][4] / scale[1];
        double kG = normal[2][4] / scale[2], kS = normal[3][4] / scale[3];
        if (!(Double.isFinite(kV) && Double.isFinite(kA) && Double.isFinite(kG) && Double.isFinite(kS)
                && kV > 0 && kA > 0 && kG >= 0 && kG < config.maxVolts() && kS >= -0.1 && kS < 2))
            throw new IllegalStateException("Invalid identified elevator model");
        kS = Math.max(0, kS);
        double errorSquared = 0;
        for (Sample s : samples) {
            double error = s.volts() - (kV*s.v() + kA*s.acceleration() + kG + kS*s.direction());
            errorSquared += error * error;
        }
        double rmse = Math.sqrt(errorSquared / samples.size());
        if (rmse > 0.3) throw new IllegalStateException("Poor elevator model fit");
        double lambda = config.bandwidthRadPerSec();
        // Closed-loop poles at -lambda and -(kV/kA + lambda), without negative damping gains.
        double kP = kV * lambda + kA * lambda * lambda;
        double kD = 2 * kA * lambda;
        if (!Double.isFinite(kP) || !Double.isFinite(kD) || kP <= 0 || kD < 0)
            throw new IllegalStateException("Invalid calculated position gains");
        return new Gains(kP, 0, kD, kV, kA, kG, kS, rmse);
    }
    @Override public void periodic() {
        if (!DriverStation.isTestEnabled()) { if (mode != Mode.OFF) stop(); return; }
        if (mode == Mode.OFF) return;
        Inputs in = io.readInputs();
        if (!safe(in)) { abort("Travel, velocity, current, or sensor safety limit"); return; }
        SmartDashboard.putNumber(dashboardPrefix + "/HeightMeters", in.heightMeters());
        double volts;
        if (mode == Mode.RUN) {
            double error = targetHeight - in.heightMeters();
            SmartDashboard.putNumber(dashboardPrefix + "/TargetHeightMeters", targetHeight);
            volts = candidate.kG()
                + candidate.kP() * error - candidate.kD() * in.velocityMetersPerSec()
                + (Math.abs(error) > 0.003 ? candidate.kS() * Math.signum(error) : 0);
        } else {
            double now = Timer.getFPGATimestamp(), dt = now - previousTime;
            if (dt < 0.015 || dt > 0.025) { abort("Identification loop must run at 20 ms"); return; }
            collect(in, dt);
            minObserved = Math.min(minObserved, in.heightMeters());
            maxObserved = Math.max(maxObserved, in.heightMeters());
            previous = in;
            previousTime = now;
            double elapsed = now - startTime;
            if (elapsed >= 12) {
                stop();
                try {
                    candidate = calculateGains();
                    SmartDashboard.putNumber(dashboardPrefix + "/kP", candidate.kP());
                    SmartDashboard.putNumber(dashboardPrefix + "/kI", candidate.kI());
                    SmartDashboard.putNumber(dashboardPrefix + "/kD", candidate.kD());
                    SmartDashboard.putNumber(dashboardPrefix + "/kV", candidate.kV());
                    SmartDashboard.putNumber(dashboardPrefix + "/kA", candidate.kA());
                    SmartDashboard.putNumber(dashboardPrefix + "/kG", candidate.kG());
                    SmartDashboard.putNumber(dashboardPrefix + "/kS", candidate.kS());
                    SmartDashboard.putNumber(dashboardPrefix + "/ModelRMSEVolts", candidate.rmseVolts());
                    publishStatus("Candidate gains ready");
                } catch (IllegalStateException ex) {
                    candidate = null;
                    publishStatus("Identification failed: " + ex.getMessage());
                }
                return;
            }
            double target = startHeight + config.sweepAmplitudeMeters() * Math.sin(2 * Math.PI * 0.25 * elapsed);
            double targetVelocity = config.sweepAmplitudeMeters() * 2 * Math.PI * 0.25 * Math.cos(2 * Math.PI * 0.25 * elapsed);
            volts = config.bootstrapKp() * (target - in.heightMeters())
                + config.bootstrapKd() * (targetVelocity - in.velocityMetersPerSec())
                + 0.7 * Math.sin(2 * Math.PI * 1.1 * elapsed);
            SmartDashboard.putNumber(dashboardPrefix + "/TuneSamples", samples.size());
        }
        volts = MathUtil.clamp(volts, -config.maxVolts(), config.maxVolts());
        io.setVoltage(volts);
        SmartDashboard.putNumber(dashboardPrefix + "/CommandVolts", volts);
    }
}


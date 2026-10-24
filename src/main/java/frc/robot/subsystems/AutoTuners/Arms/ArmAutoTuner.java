package frc.robot.subsystems.AutoTuners.Arms;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Test-mode arm identification and position control. Angles are radians from horizontal.
 * Positive voltage must increase angle. The caller must exclusively own the motor, establish
 * an absolute/homed angle, configure hardware current limits, and select safe Config values.
 */
public final class ArmAutoTuner extends SubsystemBase {
    public record Inputs(double angleRad, double velocityRadPerSec, double appliedVolts,
                         double currentAmps, boolean connected) {}
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
    public record Config(double minAngleRad, double maxAngleRad, double limitMarginRad,
                         double maxVelocityRadPerSec, double maxCurrentAmps, double maxVolts,
                         double sweepAmplitudeRad, double bootstrapKp, double bootstrapKd,
                         double bandwidthRadPerSec) {
        public Config {
            double[] values = {minAngleRad, maxAngleRad, limitMarginRad, maxVelocityRadPerSec,
                maxCurrentAmps, maxVolts, sweepAmplitudeRad, bootstrapKp, bootstrapKd, bandwidthRadPerSec};
            for (double value : values) if (!Double.isFinite(value))
                throw new IllegalArgumentException("Configuration must be finite");
            if (!(minAngleRad < maxAngleRad && limitMarginRad > 0
                    && 2 * limitMarginRad < maxAngleRad - minAngleRad
                    && maxVelocityRadPerSec > 0 && maxCurrentAmps > 0
                    && maxVolts > 0 && maxVolts <= 12 && sweepAmplitudeRad >= 0.15
                    && bootstrapKp > 0 && bootstrapKd >= 0
                    && bandwidthRadPerSec > 0 && bandwidthRadPerSec <= 5))
                throw new IllegalArgumentException("Invalid arm tuning configuration");
        }
    }
    /** Voltage units: kP V/rad, kD V/(rad/s), kV V/(rad/s), kA V/(rad/s²).
     * kG and kS are volts. kI is zero: gravity is handled by feedforward.
     */
    public record Gains(double kP, double kI, double kD, double kV, double kA,
                        double kG, double kS, double rmseVolts) {}
    private enum Mode { OFF, IDENTIFY, RUN }
    private record Sample(double v, double acceleration, double cosine, double direction, double volts) {}
    private final IO io;
    private final Config config;
    private final String dashboardPrefix;
    private final List<Sample> samples = new ArrayList<>();
    private Mode mode = Mode.OFF;
    private Gains candidate;
    private String status = "Idle";
    private Inputs previous;
    private double previousTime, startTime, startAngle, targetAngle;
    private double minObserved, maxObserved;
    private double windowTime, integralV, integralCos, integralSign, integralVolts, initialVelocity;
    private int windowTicks;

    public ArmAutoTuner(IO io, Config config) {
        this(io, config, "Arm");
    }
    public ArmAutoTuner(IO io, Config config, String dashboardPrefix) {
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
    private void publishStatus(String message) {
        status = message;
        SmartDashboard.putString(dashboardPrefix + "/TuneStatus", message);
    }
    private boolean safe(Inputs in) {
        return in != null && in.connected() && Double.isFinite(in.angleRad())
            && Double.isFinite(in.velocityRadPerSec()) && Double.isFinite(in.appliedVolts())
            && Double.isFinite(in.currentAmps()) && Math.abs(in.currentAmps()) <= config.maxCurrentAmps()
            && Math.abs(in.velocityRadPerSec()) <= config.maxVelocityRadPerSec()
            && in.angleRad() > config.minAngleRad() + config.limitMarginRad()
            && in.angleRad() < config.maxAngleRad() - config.limitMarginRad();
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
        if (!safe(in)) { abort("Unsafe or invalid arm sensors"); return; }
        if (Math.abs(in.velocityRadPerSec()) > 0.1) { stop(); publishStatus("Arm must be stopped first"); return; }
        double clearance = Math.min(in.angleRad() - config.minAngleRad(), config.maxAngleRad() - in.angleRad());
        if (clearance < config.sweepAmplitudeRad() + 2 * config.limitMarginRad()) {
            stop(); publishStatus("Not enough travel for identification sweep"); return;
        }
        io.setVoltage(0);
        samples.clear();
        candidate = null;
        previous = in;
        startAngle = in.angleRad();
        minObserved = maxObserved = startAngle;
        startTime = previousTime = Timer.getFPGATimestamp();
        resetWindow(in.velocityRadPerSec());
        mode = Mode.IDENTIFY;
        publishStatus("Collecting arm data");
    }
    public void runCandidate(double angleRad) {
        if (!Double.isFinite(angleRad) || angleRad <= config.minAngleRad() + 2 * config.limitMarginRad()
                || angleRad >= config.maxAngleRad() - 2 * config.limitMarginRad()) {
            stop(); publishStatus("Invalid test angle"); return;
        }
        if (!DriverStation.isTestEnabled() || candidate == null || mode == Mode.IDENTIFY) {
            stop(); publishStatus("No valid candidate or not in Test mode"); return;
        }
        if (!safe(io.readInputs())) { abort("Unsafe or invalid arm sensors"); return; }
        targetAngle = angleRad;
        mode = Mode.RUN;
        publishStatus("Testing candidate position gains");
    }
    private void resetWindow(double velocity) {
        windowTicks = 0;
        windowTime = integralV = integralCos = integralSign = integralVolts = 0;
        initialVelocity = velocity;
    }
    private void collect(Inputs in, double dt) {
        // Integrate over 100 ms instead of differentiating noisy individual velocity readings.
        double v = (previous.velocityRadPerSec() + in.velocityRadPerSec()) / 2;
        integralV += in.angleRad() - previous.angleRad();
        integralCos += (Math.cos(previous.angleRad()) + Math.cos(in.angleRad())) / 2 * dt;
        integralSign += Math.signum(v) * dt;
        // readInputs reports voltage actually applied during the interval that just elapsed.
        integralVolts += in.appliedVolts() * dt;
        windowTime += dt;
        if (++windowTicks == 5) {
            samples.add(new Sample(integralV / windowTime,
                (in.velocityRadPerSec() - initialVelocity) / windowTime,
                integralCos / windowTime, integralSign / windowTime, integralVolts / windowTime));
            resetWindow(in.velocityRadPerSec());
        }
    }
    private Gains calculateGains() {
        if (samples.size() < 80 || maxObserved - minObserved < 0.25)
            throw new IllegalStateException("Not enough arm motion or valid samples");
        // u = kV*velocity + kA*acceleration + kG*cos(angle) + kS*sign(velocity).
        double[][] normal = new double[4][5];
        for (Sample s : samples) {
            double[] x = {s.v(), s.acceleration(), s.cosine(), s.direction()};
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
            throw new IllegalStateException("Invalid identified arm model");
        kS = Math.max(0, kS);
        double errorSquared = 0;
        for (Sample s : samples) {
            double error = s.volts() - (kV*s.v() + kA*s.acceleration() + kG*s.cosine() + kS*s.direction());
            errorSquared += error * error;
        }
        double rmse = Math.sqrt(errorSquared / samples.size());
        if (rmse > 0.3) throw new IllegalStateException("Poor arm model fit");
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
        SmartDashboard.putNumber(dashboardPrefix + "/AngleRad", in.angleRad());
        double volts;
        if (mode == Mode.RUN) {
            double error = targetAngle - in.angleRad();
            SmartDashboard.putNumber(dashboardPrefix + "/TargetAngleRad", targetAngle);
            volts = candidate.kG() * Math.cos(in.angleRad())
                + candidate.kP() * error - candidate.kD() * in.velocityRadPerSec()
                + (Math.abs(error) > 0.02 ? candidate.kS() * Math.signum(error) : 0);
        } else {
            double now = Timer.getFPGATimestamp(), dt = now - previousTime;
            if (dt < 0.015 || dt > 0.025) { abort("Identification loop must run at 20 ms"); return; }
            collect(in, dt);
            minObserved = Math.min(minObserved, in.angleRad());
            maxObserved = Math.max(maxObserved, in.angleRad());
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
            double target = startAngle + config.sweepAmplitudeRad() * Math.sin(2 * Math.PI * 0.25 * elapsed);
            double targetVelocity = config.sweepAmplitudeRad() * 2 * Math.PI * 0.25 * Math.cos(2 * Math.PI * 0.25 * elapsed);
            volts = config.bootstrapKp() * (target - in.angleRad())
                + config.bootstrapKd() * (targetVelocity - in.velocityRadPerSec())
                + 0.7 * Math.sin(2 * Math.PI * 1.1 * elapsed);
            SmartDashboard.putNumber(dashboardPrefix + "/TuneSamples", samples.size());
        }
        volts = MathUtil.clamp(volts, -config.maxVolts(), config.maxVolts());
        io.setVoltage(volts);
        SmartDashboard.putNumber(dashboardPrefix + "/CommandVolts", volts);
    }
}

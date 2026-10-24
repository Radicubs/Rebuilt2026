package frc.robot.subsystems.AutoTuners.Arms;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Gravity-enabled physics adapter; the same tuner IO can wrap an existing real motor. */
public final class ArmAutoTunerIOSim implements ArmAutoTuner.IO {
    private final SingleJointedArmSim sim;
    private double volts;
    private final double frictionVolts;
    private final double gravityVolts;
    private final java.util.Random random = new java.util.Random(2026);
    private double angleNoiseStd, velocityNoiseStd, angleNoise, velocityNoise;
    private ArmAutoTuner.Inputs override;
    public ArmAutoTunerIOSim(double inertia, double gearing, double length,
                            double minAngle, double maxAngle, double initialAngle, double frictionVolts) {
        this(DCMotor.getNEO(1), inertia, gearing, length, minAngle, maxAngle, initialAngle, frictionVolts);
    }
    public ArmAutoTunerIOSim(DCMotor motor, double inertia, double gearing, double length,
                            double minAngle, double maxAngle, double initialAngle, double frictionVolts) {
        for (double value : new double[] {inertia, gearing, length, minAngle, maxAngle, initialAngle, frictionVolts})
            if (!Double.isFinite(value)) throw new IllegalArgumentException("Finite physics values required");
        if (motor == null || inertia <= 0 || gearing <= 0 || length <= 0 || minAngle >= maxAngle
                || initialAngle < minAngle || initialAngle > maxAngle || frictionVolts < 0)
            throw new IllegalArgumentException("Invalid arm physics configuration");
        sim = new SingleJointedArmSim(motor, gearing, inertia, length, minAngle, maxAngle, true, initialAngle);
        this.frictionVolts = frictionVolts;
        gravityVolts = (3 * inertia / (length * length)) * 9.8 * length / 2
            * motor.rOhms / (gearing * motor.KtNMPerAmp);
    }
    @Override public ArmAutoTuner.Inputs readInputs() {
        return override != null ? override : new ArmAutoTuner.Inputs(sim.getAngleRads() + angleNoise,
            sim.getVelocityRadPerSec() + velocityNoise, volts, sim.getCurrentDrawAmps(), true);
    }
    @Override public void setVoltage(double volts) {
        if (!Double.isFinite(volts) || Math.abs(volts) > 12)
            throw new IllegalArgumentException("Finite voltage within 12 V required");
        this.volts = volts;
    }
    public double getCommandVolts() { return volts; }
    public void setSensorOverride(ArmAutoTuner.Inputs value) { override = value; }
    public void setState(double angle, double velocity) {
        sim.setState(angle, velocity);
        angleNoise = velocityNoise = 0;
    }
    public void setSensorNoise(double angleStdRad, double velocityStdRadPerSec) {
        angleNoiseStd = angleStdRad;
        velocityNoiseStd = velocityStdRadPerSec;
    }
    public void update(double dt) {
        // Small substeps model friction direction changes without a 20 ms sign delay.
        for (int i = 0; i < 20; i++) {
            double velocity = sim.getVelocityRadPerSec();
            double direction = Math.abs(velocity) > 0.005 ? Math.signum(velocity)
                : Math.signum(volts - gravityVolts * Math.cos(sim.getAngleRads()));
            sim.setInputVoltage(volts - frictionVolts * direction);
            sim.update(dt / 20);
        }
        angleNoise = random.nextGaussian() * angleNoiseStd;
        velocityNoise = random.nextGaussian() * velocityNoiseStd;
    }
}


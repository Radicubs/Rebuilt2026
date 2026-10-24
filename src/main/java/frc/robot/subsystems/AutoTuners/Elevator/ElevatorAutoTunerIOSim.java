package frc.robot.subsystems.AutoTuners.Elevator;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Standalone gravity-enabled elevator physics, including optional Coulomb friction and noise. */
public final class ElevatorAutoTunerIOSim implements ElevatorAutoTuner.IO {
    private final ElevatorSim sim;
    private final double frictionVolts, gravityVolts;
    private final java.util.Random random = new java.util.Random(2026);
    private double volts, heightNoiseStd, velocityNoiseStd, heightNoise, velocityNoise;
    private ElevatorAutoTuner.Inputs override;

    public ElevatorAutoTunerIOSim(DCMotor motor, double massKg, double gearing, double drumRadiusMeters,
            double minHeightMeters, double maxHeightMeters, double initialHeightMeters, double frictionVolts) {
        for (double value : new double[] {massKg, gearing, drumRadiusMeters, minHeightMeters,
                maxHeightMeters, initialHeightMeters, frictionVolts})
            if (!Double.isFinite(value)) throw new IllegalArgumentException("Finite physics values required");
        if (motor == null || massKg <= 0 || gearing <= 0 || drumRadiusMeters <= 0
                || minHeightMeters >= maxHeightMeters || initialHeightMeters < minHeightMeters
                || initialHeightMeters > maxHeightMeters || frictionVolts < 0)
            throw new IllegalArgumentException("Invalid elevator physics configuration");
        sim = new ElevatorSim(motor, gearing, massKg, drumRadiusMeters,
            minHeightMeters, maxHeightMeters, true, initialHeightMeters);
        this.frictionVolts = frictionVolts;
        gravityVolts = massKg * 9.8 * drumRadiusMeters * motor.rOhms / (gearing * motor.KtNMPerAmp);
    }
    @Override public ElevatorAutoTuner.Inputs readInputs() {
        return override != null ? override : new ElevatorAutoTuner.Inputs(
            sim.getPositionMeters() + heightNoise, sim.getVelocityMetersPerSecond() + velocityNoise,
            volts, sim.getCurrentDrawAmps(), true, true, sim.hasHitLowerLimit(), sim.hasHitUpperLimit());
    }
    @Override public void setVoltage(double volts) {
        if (!Double.isFinite(volts) || Math.abs(volts) > 12)
            throw new IllegalArgumentException("Finite voltage within 12 V required");
        this.volts = volts;
    }
    public double getCommandVolts() { return volts; }
    public void setSensorOverride(ElevatorAutoTuner.Inputs value) { override = value; }
    public void setState(double heightMeters, double velocityMetersPerSec) {
        sim.setState(heightMeters, velocityMetersPerSec);
        heightNoise = velocityNoise = 0;
    }
    public void setSensorNoise(double heightStdMeters, double velocityStdMetersPerSec) {
        if (!Double.isFinite(heightStdMeters) || !Double.isFinite(velocityStdMetersPerSec)
                || heightStdMeters < 0 || velocityStdMetersPerSec < 0)
            throw new IllegalArgumentException("Finite nonnegative noise required");
        heightNoiseStd = heightStdMeters;
        velocityNoiseStd = velocityStdMetersPerSec;
    }
    public void update(double dt) {
        if (!Double.isFinite(dt) || dt <= 0) throw new IllegalArgumentException("Positive finite dt required");
        for (int i = 0; i < 20; i++) {
            double velocity = sim.getVelocityMetersPerSecond();
            double direction = Math.abs(velocity) > 0.002 ? Math.signum(velocity) : Math.signum(volts - gravityVolts);
            sim.setInputVoltage(volts - frictionVolts * direction);
            sim.update(dt / 20);
        }
        heightNoise = random.nextGaussian() * heightNoiseStd;
        velocityNoise = random.nextGaussian() * velocityNoiseStd;
    }
}



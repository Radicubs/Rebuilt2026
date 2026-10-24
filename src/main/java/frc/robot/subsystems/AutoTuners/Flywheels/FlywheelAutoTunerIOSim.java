package frc.robot.subsystems.AutoTuners.Flywheels;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Standalone WPILib physics IO. No vendor JNI, CAN device, or robot constants required. */
public final class FlywheelAutoTunerIOSim implements FlywheelAutoTuner.IO {
    private final FlywheelSim sim;
    private final double supplyVolts;
    private double volts;
    private FlywheelAutoTuner.Inputs override;
    public FlywheelAutoTunerIOSim(DCMotor gearbox, double inertiaKgM2, double gearing, double supplyVolts) {
        if (gearbox == null || !Double.isFinite(inertiaKgM2) || inertiaKgM2 <= 0
                || !Double.isFinite(gearing) || gearing <= 0
                || !Double.isFinite(supplyVolts) || supplyVolts <= 0 || supplyVolts > 12)
            throw new IllegalArgumentException("Motor, positive inertia/gearing, and supply up to 12 V required");
        this.supplyVolts = supplyVolts;
        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox, inertiaKgM2, gearing), gearbox);
    }
    @Override public FlywheelAutoTuner.Inputs readInputs() {
        return override != null ? override : new FlywheelAutoTuner.Inputs(
            sim.getAngularVelocityRPM()/60, volts,
            Math.abs(sim.getCurrentDrawAmps() * volts / supplyVolts), true);
    }
    @Override public void setVoltage(double volts) {
        if (!Double.isFinite(volts) || Math.abs(volts) > supplyVolts)
            throw new IllegalArgumentException("Voltage exceeds simulated supply");
        this.volts = volts;
        sim.setInputVoltage(volts);
    }
    public void update(double dt) { sim.update(dt); }
    public void setVelocityRPS(double rps) { sim.setAngularVelocity(rps * 2 * Math.PI); }
    public double getCommandVolts() { return volts; }
    public void setSensorOverride(FlywheelAutoTuner.Inputs value) { override = value; }
}


package frc.robot.subsystems.intake;

import frc.robot.constants.IntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

class IntakeIOSim implements IntakeIO {

    private static final double NOMINAL_BUS_VOLTAGE = 12.0;

    private final DCMotor gearbox = DCMotor.getNEO(1);
    private final FlywheelSim flywheel;
    private double appliedVolts = 0.0;

    IntakeIOSim() {
        LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
                gearbox,
                IntakeConstants.SimConstants.momentOfInertiaKgMetersSquared,
                IntakeConstants.SimConstants.gearing);
        flywheel = new FlywheelSim(plant, gearbox);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        flywheel.update(0.02);
        inputs.velocityRPS = flywheel.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = flywheel.getCurrentDrawAmps();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        appliedVolts = MathUtil.clamp(dutyCycle, -1.0, 1.0) * NOMINAL_BUS_VOLTAGE;
        flywheel.setInputVoltage(appliedVolts);
    }
}

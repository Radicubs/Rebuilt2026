package frc.robot.subsystems.pivot;

import frc.robot.constants.PivotConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

class PivotIOSim implements PivotIO {

    private static final double NOMINAL_BUS_VOLTAGE = 12.0;

    private final DCMotor gearbox = DCMotor.getNEO(1);
    private final DCMotorSim sim;
    private double lastDuty = 0.0;

    PivotIOSim() {
        LinearSystem<N2, N1, N2> plant = LinearSystemId.createDCMotorSystem(
                gearbox,
                PivotConstants.SimConstants.momentOfInertiaKgMetersSquared,
                PivotConstants.SimConstants.gearing);
        sim = new DCMotorSim(plant, gearbox);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        sim.update(0.02);
        inputs.positionRot = sim.getAngularPositionRotations();
        inputs.appliedDuty = lastDuty;
        inputs.appliedVolts = lastDuty * NOMINAL_BUS_VOLTAGE;
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        lastDuty = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        sim.setInputVoltage(lastDuty * NOMINAL_BUS_VOLTAGE);
    }

    @Override
    public void setPosition(double positionRot) {
        sim.setState(Units.rotationsToRadians(positionRot), 0);
    }
}

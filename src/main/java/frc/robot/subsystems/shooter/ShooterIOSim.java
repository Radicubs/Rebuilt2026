package frc.robot.subsystems.shooter;

import frc.robot.constants.ShooterConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

class ShooterIOSim implements ShooterIO {

    private static final double DT = 0.02;

    private static final class Flywheel {
        private final FlywheelSim sim;
        private final PIDController pid;
        private final SimpleMotorFeedforward ff;
        private double targetRps;
        private double appliedVolts;

        Flywheel(double moiKgM2, double kP, double kI, double kD, double kS, double kV, double kA) {
            DCMotor gearbox = DCMotor.getFalcon500(1);
            sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox, moiKgM2, 1.0), gearbox);
            pid = new PIDController(kP, kI, kD);
            ff = new SimpleMotorFeedforward(kS, kV, kA);
        }

        void setTarget(double rps) {
            targetRps = rps;
        }

        double update() {
            double rps = sim.getAngularVelocityRPM() / 60.0;
            appliedVolts = MathUtil.clamp(pid.calculate(rps, targetRps) + ff.calculate(targetRps), -12.0, 12.0);
            sim.setInputVoltage(appliedVolts);
            sim.update(DT);
            return sim.getAngularVelocityRPM() / 60.0;
        }
    }

    private final Flywheel left = new Flywheel(
            ShooterConstants.SimConstants.mainShooterMoiKgM2,
            ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kP,
            ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kI,
            ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kD,
            ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kS,
            ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kV,
            ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kA);

    private final Flywheel right = new Flywheel(
            ShooterConstants.SimConstants.mainShooterMoiKgM2,
            ShooterConstants.MainRightShooterPIDFeedforwardConstants.kP,
            ShooterConstants.MainRightShooterPIDFeedforwardConstants.kI,
            ShooterConstants.MainRightShooterPIDFeedforwardConstants.kD,
            ShooterConstants.MainRightShooterPIDFeedforwardConstants.kS,
            ShooterConstants.MainRightShooterPIDFeedforwardConstants.kV,
            ShooterConstants.MainRightShooterPIDFeedforwardConstants.kA);


    private final Flywheel top = new Flywheel(
            ShooterConstants.SimConstants.topShooterMoiKgM2,
            ShooterConstants.TopShooterPIDFeedforwardConstants.kP, ShooterConstants.TopShooterPIDFeedforwardConstants.kI,
            ShooterConstants.TopShooterPIDFeedforwardConstants.kD, ShooterConstants.TopShooterPIDFeedforwardConstants.kS,
            ShooterConstants.TopShooterPIDFeedforwardConstants.kV, ShooterConstants.TopShooterPIDFeedforwardConstants.kA);
    private final Flywheel indexer = new Flywheel(
            ShooterConstants.SimConstants.indexerMoiKgM2,
            ShooterConstants.IndexerPIDFeedforwardConstants.kP, ShooterConstants.IndexerPIDFeedforwardConstants.kI,
            ShooterConstants.IndexerPIDFeedforwardConstants.kD, ShooterConstants.IndexerPIDFeedforwardConstants.kS,
            ShooterConstants.IndexerPIDFeedforwardConstants.kV, ShooterConstants.IndexerPIDFeedforwardConstants.kA);


    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftVelocityRps = left.update();
        inputs.rightVelocityRps = right.update();
        inputs.topVelocityRps = top.update();
        inputs.indexerVelocityRps = indexer.update();

        inputs.leftAppliedVolts = left.appliedVolts;
        inputs.rightAppliedVolts = right.appliedVolts;
        inputs.topAppliedVolts = top.appliedVolts;
        inputs.indexerAppliedVolts = indexer.appliedVolts;

        inputs.leftCurrentAmps = left.sim.getCurrentDrawAmps();
        inputs.rightCurrentAmps = right.sim.getCurrentDrawAmps();
        inputs.topCurrentAmps = top.sim.getCurrentDrawAmps();
        inputs.indexerCurrentAmps = indexer.sim.getCurrentDrawAmps();
    }

    @Override
    public void setShooterSpeeds(double mainRps, double topRps, double indexerRps) {
        left.setTarget(mainRps);
        right.setTarget(mainRps);
        top.setTarget(topRps);
        indexer.setTarget(indexerRps);
    }

    @Override
    public void setIndexerSpeed(double indexerRps) {
        indexer.setTarget(indexerRps);
    }

    @Override
    public void stop() {
        left.setTarget(0);
        right.setTarget(0);
        top.setTarget(0);
        indexer.setTarget(0);
    }
}

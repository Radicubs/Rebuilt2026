package frc.robot.subsystems.drive.module;

import frc.robot.constants.DriveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.Conversions;

class ModuleIOSim implements ModuleIO {

    private static final double DT = 0.02;

    private final DCMotorSim driveSim;
    private final DCMotorSim angleSim;

    private final SimpleMotorFeedforward driveFeedForward =
            new SimpleMotorFeedforward(DriveConstants.driveKS, DriveConstants.driveKV, DriveConstants.driveKA);
    private final PIDController driveController =
            new PIDController(DriveConstants.driveKP, DriveConstants.driveKI, DriveConstants.driveKD);
    private final PIDController angleController =
            new PIDController(DriveConstants.angleKP, DriveConstants.angleKI, DriveConstants.angleKD);

    private boolean driveClosedLoop = false;
    private double driveFFVolts = 0.0;
    private double driveSetpointRps = 0.0;
    private double driveOpenLoopVolts = 0.0;
    private double driveAppliedVolts = 0.0;

    private double angleSetpointRot = 0.0;
    private double angleAppliedVolts = 0.0;

    ModuleIOSim() {
        DCMotor driveGearbox = DCMotor.getKrakenX60(1);
        DCMotor angleGearbox = DCMotor.getKrakenX60(1);
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(driveGearbox, DriveConstants.SimConstants.driveMoiKgM2, DriveConstants.driveGearRatio),
                driveGearbox);
        angleSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(angleGearbox, DriveConstants.SimConstants.angleMoiKgM2, DriveConstants.angleGearRatio),
                angleGearbox);
        angleController.enableContinuousInput(-0.5, 0.5);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (driveClosedLoop) {
            double wheelRps = driveSim.getAngularVelocityRPM() / 60.0;
            driveAppliedVolts = MathUtil.clamp(driveFFVolts + driveController.calculate(wheelRps, driveSetpointRps), -12.0, 12.0);
        } else {
            driveController.reset();
            driveAppliedVolts = MathUtil.clamp(driveOpenLoopVolts, -12.0, 12.0);
        }
        driveSim.setInputVoltage(driveAppliedVolts);
        driveSim.update(DT);

        double moduleRot = angleSim.getAngularPositionRotations();
        angleAppliedVolts = MathUtil.clamp(angleController.calculate(moduleRot, angleSetpointRot), -12.0, 12.0);
        angleSim.setInputVoltage(angleAppliedVolts);
        angleSim.update(DT);

        inputs.drivePositionMeters = Conversions.rotationsToMeters(
                driveSim.getAngularPositionRotations(), DriveConstants.wheelCircumference);
        inputs.driveVelocityMps = Conversions.RPSToMPS(
                driveSim.getAngularVelocityRPM() / 60.0, DriveConstants.wheelCircumference);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.anglePosition = Rotation2d.fromRotations(angleSim.getAngularPositionRotations());
        inputs.angleAbsolutePosition = inputs.anglePosition;
        inputs.angleVelocityRps = angleSim.getAngularVelocityRPM() / 60.0;
        inputs.angleAppliedVolts = angleAppliedVolts;
        inputs.angleCurrentAmps = Math.abs(angleSim.getCurrentDrawAmps());
    }

    @Override
    public void setDriveVelocity(double metersPerSecond) {
        driveClosedLoop = true;
        driveSetpointRps = Conversions.MPSToRPS(metersPerSecond, DriveConstants.wheelCircumference);
        driveFFVolts = driveFeedForward.calculate(metersPerSecond);
    }

    @Override
    public void setDriveDutyCycle(double dutyCycle) {
        driveClosedLoop = false;
        driveOpenLoopVolts = dutyCycle * 12.0;
    }

    @Override
    public void setAnglePosition(Rotation2d angle) {
        angleSetpointRot = angle.getRotations();
    }

    @Override
    public void stop() {
        driveClosedLoop = false;
        driveOpenLoopVolts = 0.0;
    }
}

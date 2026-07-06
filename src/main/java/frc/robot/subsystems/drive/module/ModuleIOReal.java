package frc.robot.subsystems.drive.module;

import frc.robot.constants.SwerveModuleConstants;
import frc.robot.constants.DriveConstants;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.Conversions;

class ModuleIOReal implements ModuleIO {

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder angleEncoder;
    private final Rotation2d angleOffset;

    private final SimpleMotorFeedforward driveFeedForward =
            new SimpleMotorFeedforward(DriveConstants.driveKS, DriveConstants.driveKV, DriveConstants.driveKA);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final PositionVoltage anglePositionP = new PositionVoltage(0);
    private final MotionMagicVoltage anglePositionM = new MotionMagicVoltage(0);

    ModuleIOReal(SwerveModuleConstants moduleConstants) {
        angleOffset = moduleConstants.angleOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(moduleConstants.ctreConfigs.swerveCANcoderConfig);

        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        angleMotor.getConfigurator().apply(moduleConstants.ctreConfigs.swerveAngleFXConfig);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
        resetToAbsolute();

        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        driveMotor.getConfigurator().apply(moduleConstants.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.getConfigurator().setPosition(0.0);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionMeters = Conversions.rotationsToMeters(
                driveMotor.getPosition().getValueAsDouble(), DriveConstants.wheelCircumference);
        inputs.driveVelocityMps = Conversions.RPSToMPS(
                driveMotor.getVelocity().getValueAsDouble(), DriveConstants.wheelCircumference);
        inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveCurrentAmps = driveMotor.getSupplyCurrent().getValueAsDouble();

        inputs.anglePosition = Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
        inputs.angleAbsolutePosition = Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
        inputs.angleVelocityRps = angleMotor.getVelocity().getValueAsDouble();
        inputs.angleAppliedVolts = angleMotor.getMotorVoltage().getValueAsDouble();
        inputs.angleCurrentAmps = angleMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setDriveVelocity(double metersPerSecond) {
        double rps = Conversions.MPSToRPS(metersPerSecond, DriveConstants.wheelCircumference);
        driveMotor.setControl(driveVelocity.withVelocity(rps).withFeedForward(driveFeedForward.calculate(metersPerSecond)));
    }

    @Override
    public void setDriveDutyCycle(double dutyCycle) {
        driveMotor.setControl(driveDutyCycle.withOutput(dutyCycle));
    }

    @Override
    public void setAnglePosition(Rotation2d angle) {
        if (DriveConstants.useMagicMotion) {
            angleMotor.setControl(anglePositionM.withPosition(angle.getRotations()));
        } else {
            angleMotor.setControl(anglePositionP.withPosition(angle.getRotations()));
        }
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = angleEncoder.getAbsolutePosition().getValueAsDouble() - angleOffset.getRotations();
        angleMotor.setPosition(absolutePosition);
    }

    @Override
    public void stop() {
        driveMotor.setControl(driveDutyCycle.withOutput(0));
    }
}

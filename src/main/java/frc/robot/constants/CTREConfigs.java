package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(InvertedValue driveMotorInvert, InvertedValue angleMotorInvert) {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = DriveConstants.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = DriveConstants.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = DriveConstants.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = DriveConstants.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = DriveConstants.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = DriveConstants.angleCurrentThresholdTime;

        /* PID Config */

        swerveAngleFXConfig.Slot0.kP = DriveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = DriveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = DriveConstants.angleKD;


        swerveAngleFXConfig.MotionMagic.MotionMagicCruiseVelocity = DriveConstants.magicMotionTopSpeed;
        swerveAngleFXConfig.MotionMagic.MotionMagicAcceleration = DriveConstants.magicMotionAccel;
        swerveAngleFXConfig.MotionMagic.MotionMagicJerk = DriveConstants.magicMotionJerk;


        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = DriveConstants.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = DriveConstants.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = DriveConstants.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = DriveConstants.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = DriveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = DriveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = DriveConstants.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = DriveConstants.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DriveConstants.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = DriveConstants.closedLoopRamp;
    }

    public CTREConfigs(){
        //taken from default COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2
        this(InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive);
    }
}
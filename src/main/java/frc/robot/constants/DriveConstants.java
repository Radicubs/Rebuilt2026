package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final COTSTalonFXSwerveConstants chosenModule =  // MK4i with Kraken X60 drive motors
            COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(26); //TODO: This must be tuned to specific robot
    public static final double wheelBase = Units.inchesToMeters(26); //TODO: This must be tuned to specific robot
    public static final double centerDistance = trackWidth / 2.0;
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(centerDistance, centerDistance),
            new Translation2d(centerDistance, -centerDistance),
            new Translation2d(-centerDistance, centerDistance),
            new Translation2d(-centerDistance, -centerDistance));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 15;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 30;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Lock on values*/
    public static final double lockKP = 0.9; //TODO: This must be tuned to specific robot
    public static final double lockDeadband = 0.025;
    public static final double lockOnMaxSpeed = 2;

    /* Angle Motor Magic Motion Values */
    public static final double magicMotionAccel = 18;
    public static final double magicMotionJerk = 130;
    public static final double magicMotionTopSpeed = 100;
    public static final boolean useMagicMotion = false;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.2; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.218; //TODO: This must be tuned to specific robot
    public static final double driveKV = 2.36; //2.1 overshoots
    public static final double driveKA = 0.0;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 1.3; //Old 3.9 TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 4.5; //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    // sim-only per-module moments of inertia
    public static final class SimConstants {
        public static final double driveMoiKgM2 = 0.01;   // wheel + reflected motor inertia
        public static final double angleMoiKgM2 = 0.004;  // steer inertia at the module
    }

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 6;
        public static final int angleMotorID = 5;
        public static final int canCoderID = 11;
        public static InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
        public static InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-47.197);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleMotorInvert, driveMotorInvert);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 8;
        public static final int angleMotorID = 7;
        public static final int canCoderID = 12;
        public static InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
        public static InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(56.338);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleMotorInvert, driveMotorInvert);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 4;
        public static final int angleMotorID = 3;
        public static final int canCoderID = 10;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-173.320);
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 2;
        public static final int angleMotorID = 1;
        public static final int canCoderID = 9;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-27.246);
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
}

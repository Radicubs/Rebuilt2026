// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants
{
    public record TrajectoryConstants(
            double maxPathSpeed,
            double maxPathAngularSpeed,
            double maxPathAcceleration,
            double maxPathAngularAcceleration,

            double xControllerkP,
            double xControllerkI,
            double xControllerkD,

            double yControllerkP,
            double yControllerkI,
            double yControllerkD,

            double thetaControllerkP,
            double thetaControllerkI,
            double thetaControllerkD,

            double xTolerance,
            double yTolerance,
            double rotTolerance,
            double velXTolerance,
            double velYTolerance,
            double rotVelTolerance
    ) {
        public static final frc.robot.Constants.TrajectoryConstants DEFAULTS = new frc.robot.Constants.TrajectoryConstants(4, 10,
                3.0, 14.0, 2.5, 0, 0,
                2.5, 0, 0, 1.5, 0, 0,
                0.05, 0.05, 0.05, 0.12, 0.12, 0.12);

        public static final frc.robot.Constants.TrajectoryConstants SLOW = new frc.robot.Constants.TrajectoryConstants(1, 10,
                3.0, 14.0, 0.75, 0, 0,
                0.75, 0, 0, 0.365, 0, 0,
                0.015, 0.015, 0.035, 0.1, 0.1, 0.1);
    }

    public static final class Field {
        /* Starting Positions */
        public static final Pose2d rightStart = new Pose2d(3.651, 0.613, new Rotation2d());
        public static final Pose2d middleStart =  new Pose2d(3.651, 3.964, new Rotation2d());
        public static final Pose2d leftStart = new Pose2d(3.651, 7.444, new Rotation2d());

        /* Positions */
        public static final Pose2d hubCenter = new Pose2d(1, 0, new Rotation2d()); // x: 4.634 y: 4.003
        public static final Pose2d hub = new Pose2d(2.318, 3.964, new Rotation2d());

        public static final Pose2d depotLeft = new Pose2d(0.429, 7.1408, new Rotation2d(90));
        public static final Pose2d depotRight = new Pose2d(0.429, 4.866, new Rotation2d(90));
        public static final Pose2d outpost = new Pose2d(0.429, 0.713, new Rotation2d(-90));

        public static final Pose2d rightLadder = new Pose2d(1.348, 2.864, new Rotation2d(180));
        public static final Pose2d middleLadder = new Pose2d(0.766, 3.731, new Rotation2d());
        public static final Pose2d leftLadder = new Pose2d(0.766, 4.546, new Rotation2d());
    }

    public static final class CameraConfig {
        public static double cameraOffsetY = 0;
        public static double cameraOffsetX = 0;
        public static double cameraOffsetZ = 0;

    }

    public static final class Swerve {
        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

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

        /* Motor Inverts */

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
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
        public static final double lockKP = 0.75; //TODO: This must be tuned to specific robot
        public static final double lockDeadband = 0.025;
        public static final double lockOnMaxSpeed = 1;

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
        public static final double driveKS = 0.2; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.925; //2.1 overshoots
        public static final double driveKA = 0.0;

        /* Swerve Profiling Values */
        /**
         * Meters per Second
         */
        public static final double maxSpeed = 2.5; //TODO: This must be tuned to specific robot
        /**
         * Radians per Second
         */
        public static final double maxAngularVelocity = 6; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
            public static InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(133.9 + 180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleMotorInvert, driveMotorInvert);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
            public static InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-122.6 + 180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleMotorInvert, driveMotorInvert);

        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(5.8 + 180);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(151.5 + 180);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Shooter{
        public static final int rightShooterCID = 13;
        public static final int leftShooterCID = 14;

        public static final int indexerCID = 15;
        public static final int topShooterCID = 16;

        public static final int shooterCurrentLimit = 10;
        public static final int shooterCurrentThreshold = 40;
        public static final double shooterCurrentThresholdTime = 0.1;
        public static final boolean shooterEnableCurrentLimit = true;

        public static double mainShooterRPS = 60;
        public static final double topShaftSpeed = .15;
        public static final double indexerSpeed = .7;
        public static double kP = 0.4;
        public static double kI = 0;
        public static double kD = -0.01;
        public static double kS = .15;
        public static double kV = 0.11;
        public static double kA = 0;

    }

    public static final class Transfer{
        public static final int beltMotorCID = 18;

        public static final double transferSpeed = .3;
    }

    public static final class Intake{
        public static final int intakeMotorCID = 19;

        public static final double intakeSpeed = .7;
        public static final class PIDFeedforwardConstants {
            public static final double P = 1.1;
            public static final double I = 0;
            public static final double D = 0;
            public static final double S = 0;
            public static final double V = 0;
        }
    }

    public static final class Pivot{
        public static final int pivotMotorCID = 17;

        public static final double downPos = 0.27;
        public static final double upPos = 0.05;

        public static final class PIDFeedforwardConstants {
            public static final double P = 1.1;
            public static final double I = 0;
            public static final double D = 0;
            public static final double S = 0;
            public static final double V = 0;
            public static final double G = .019;
        }
    }
}

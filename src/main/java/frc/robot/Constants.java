// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants
{
    public static final class CameraConfig {
        public static double cameraOffsetY = 0;
        public static double cameraOffsetX = 0;
        public static double cameraOffsetZ = 0;

    }

    public static final class Swerve {
        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(27.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(27.5); //TODO: This must be tuned to specific robot
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
        public static final double driveKS = 0.15; //TODO: This must be tuned to specific robot
        public static final double driveKV = 2.1;
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
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 9;
            public static InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
            public static InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(151.5);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleMotorInvert, driveMotorInvert);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
            public static InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(5.8);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleMotorInvert, driveMotorInvert);

        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-122.6);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(133.9);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}

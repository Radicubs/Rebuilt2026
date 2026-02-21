// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.pathcommands.AlignToTarget;
import frc.robot.commands.teleop.TeleopSwerve;
import frc.robot.subsystems.*;


public class RobotContainer {
    XboxController mainController;
    private JoystickButton mainA, mainB, mainX, mainY, rightBumper, leftBumper;
    private double deadband = 0.05;

    public RobotContainer() {
        PhotonVision.getInstance();

        mainController = new XboxController(0);
        rightBumper = new JoystickButton(mainController, XboxController.Button.kRightBumper.value);
        leftBumper = new JoystickButton(mainController, XboxController.Button.kLeftBumper.value);
        mainA = new JoystickButton(mainController, XboxController.Button.kA.value);
        mainB = new JoystickButton(mainController, XboxController.Button.kB.value);
        mainX = new JoystickButton(mainController, XboxController.Button.kX.value);
        mainY = new JoystickButton(mainController, XboxController.Button.kY.value);
        Swerve.getInstance().setDefaultCommand(new TeleopSwerve(
                () -> -mainController.getLeftY(),
                () -> -mainController.getLeftX(),
                () -> -mainController.getRightX(),
                () -> mainController.getAButton()
        ));

        configureBindings();

    }
    private void configureBindings () {
        rightBumper.onTrue(new InstantCommand(() -> {
            Swerve.getInstance().zeroHeading();
        }));

        mainA.onTrue(new InstantCommand(() -> {
            Shooter.getInstance().setShooterSpeeds(Constants.Shooter.mainShooterSpeed, Constants.Shooter.topShaftSpeed, Constants.Shooter.indexerSpeed);
        })).onFalse(new InstantCommand(() -> {
            Shooter.getInstance().setShooterSpeeds(0, 0, 0);
        }));

        mainB.onTrue(new InstantCommand(() -> {
            if(Math.abs(Pivot.getInstance().getPosition()) - deadband > 0) {
                Intake.getInstance().setIntakeSpeed(Constants.Intake.intakeSpeed);
                Transfer.getInstance().setTransferSpeeds(Constants.Transfer.transferSpeed);
            }
        })).onFalse(new InstantCommand(() -> {
            Intake.getInstance().setIntakeSpeed(0);
            Transfer.getInstance().setTransferSpeeds(0);
        }));

        mainX.onTrue(new InstantCommand(() -> {
            Pivot.getInstance().setSetpoint(Constants.Pivot.downPos);
        }));

        mainY.onTrue(new InstantCommand(() -> {
            Pivot.getInstance().setSetpoint(Constants.Pivot.upPos);
        }));

//        leftBumper.onTrue(new InstantCommand(() -> {
//            Swerve.getInstance().setPose(new Pose2d(0, 0, new Rotation2d()));
//        }));


    }


    public Command getAutonomousCommand ()
    {
        return new AlignToTarget(new Transform2d(1, 0, new Rotation2d(0)), Constants.TrajectoryConstants.SLOW);
    }
}

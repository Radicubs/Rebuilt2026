// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.pathcommands.AlignToTarget;
import frc.robot.commands.teleop.TeleopSwerve;
import frc.robot.subsystems.*;


public class RobotContainer {
    XboxController mainController;
    private JoystickButton mainA, mainB, mainX, mainY, rightBumper, leftBumper;
    private Trigger up, down, left, right, mainLT, mainRT;
    private double deadband = 0.05;

    public RobotContainer() {
        PhotonVision.getInstance();
        Pivot.getInstance();

        // Button Initialization
        {
            mainController = new XboxController(0);
            rightBumper = new JoystickButton(mainController, XboxController.Button.kRightBumper.value);
            leftBumper = new JoystickButton(mainController, XboxController.Button.kLeftBumper.value);
            mainA = new JoystickButton(mainController, XboxController.Button.kA.value);
            mainB = new JoystickButton(mainController, XboxController.Button.kB.value);
            mainX = new JoystickButton(mainController, XboxController.Button.kX.value);
            mainY = new JoystickButton(mainController, XboxController.Button.kY.value);
            up = new Trigger(() -> mainController.getPOV() == 0);
            right = new Trigger(() -> mainController.getPOV() == 90);
            down = new Trigger(() -> mainController.getPOV() == 180);
            left = new Trigger(() -> mainController.getPOV() == 270);
            mainRT = new Trigger(() -> mainController.getRightTriggerAxis() > 0.1);
            mainLT = new Trigger(() -> mainController.getLeftTriggerAxis() > 0.1);
        }

        // Named Commands
        {
            // Shooter
            NamedCommands.registerCommand("RampShooter", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(Constants.Shooter.mainShooterRPS, Constants.Shooter.topShaftRPS, 0);
            }));

            NamedCommands.registerCommand("StartShoot", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(Constants.Shooter.mainShooterRPS, Constants.Shooter.topShaftRPS, Constants.Shooter.indexerRPS);
                Transfer.getInstance().setTransferSpeed(Constants.Transfer.transferSpeed);
            }));

            NamedCommands.registerCommand("StopShoot", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(0, 0, 0);
                Transfer.getInstance().setTransferSpeed(0);
            }));

            // Intake
            NamedCommands.registerCommand("ExtendIntake", new InstantCommand(() -> {
                Pivot.getInstance().setSetpoint(Constants.Pivot.downPos);
            }));
            NamedCommands.registerCommand("RetractIntake", new InstantCommand(() -> {
                Pivot.getInstance().setSetpoint(Constants.Pivot.upPos);
            }));

            NamedCommands.registerCommand("StartIntake", new InstantCommand(() -> {
                Intake.getInstance().setIntakeSpeed(Constants.Intake.intakeSpeedRPS);
            }));

            NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {
                Intake.getInstance().setIntakeSpeed(0);
            }));
        }

        Swerve.getInstance().setDefaultCommand(new TeleopSwerve(
                () -> -mainController.getLeftY(),
                () -> -mainController.getLeftX(),
                () -> -mainController.getRightX(),
                () -> false
        ));

        configureBindings();

    }

    private void configureBindings () {

        mainRT.onTrue(new InstantCommand(() -> {
            Shooter.getInstance().setShooterSpeeds(Constants.Shooter.mainShooterRPS, Constants.Shooter.topShaftRPS, Constants.Shooter.indexerRPS);
            Transfer.getInstance().setTransferSpeed(Constants.Transfer.transferSpeed);
        })).onFalse(new InstantCommand(() -> {
            Shooter.getInstance().setShooterSpeeds(0,0, 0);
            Transfer.getInstance().setTransferSpeed(0);
        }));

        mainLT.onTrue(new InstantCommand(() -> {
            Intake.getInstance().setIntakeSpeed(Constants.Intake.intakeSpeedRPS);
            Pivot.getInstance().setSpeed(0.05);
            Transfer.getInstance().setTransferSpeed(Constants.Transfer.transferSpeed);
        })).onFalse(new InstantCommand(() -> {
            Intake.getInstance().cancelPID();
            Pivot.getInstance().setSpeed(0);
            Transfer.getInstance().setTransferSpeed(0);

        }));

        mainX.onTrue(new InstantCommand(() -> {
            Pivot.getInstance().setSetpoint(Constants.Pivot.downPos);
        }));

        mainY.onTrue(new InstantCommand(() -> {
            Pivot.getInstance().setSetpoint(Constants.Pivot.upPos);
        }));

        mainB.onTrue(new InstantCommand(() -> {
           Shooter.getInstance().setShooterSpeeds(Constants.Shooter.mainShooterRPS, Constants.Shooter.topShaftRPS, 0);
        }));

        up.onTrue(new InstantCommand(() -> {
            Pivot.getInstance().cancelPID();
            Pivot.getInstance().setSpeed(-0.3);
        })).onFalse(new InstantCommand(() -> Pivot.getInstance().setSpeed(0)));

        down.onTrue(new InstantCommand(() -> {
            Pivot.getInstance().cancelPID();
            Pivot.getInstance().setSpeed(0.3);
        })).onFalse(new InstantCommand(() -> Pivot.getInstance().setSpeed(0)));

//        leftBumper.onTrue(new InstantCommand(() -> {
//            Swerve.getInstance().setPose(new Pose2d(0, 0, new Rotation2d()));
//        }));

        leftBumper.onTrue(new InstantCommand(() -> Pivot.getInstance().zero()));

        rightBumper.onTrue(new InstantCommand(() -> {
            Swerve.getInstance().zeroHeading();
        }));
    }


    public Command getAutonomousCommand ()
    {
        return new AlignToTarget(new Transform2d(1, 0, new Rotation2d(0)), Constants.TrajectoryConstants.SLOW);
    }
}

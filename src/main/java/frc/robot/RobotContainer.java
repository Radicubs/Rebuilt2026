// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.teleop.TeleopSwerve;
import frc.robot.subsystems.*;


public class RobotContainer {

    XboxController mainController;
    private JoystickButton mainA, mainB, mainX, mainY, mainRB, mainLB;
    private Trigger mainUp, mainDown, mainLeft, mainRight, mainLT, mainRT;

    XboxController secondaryController;
    private JoystickButton secondaryA, secondaryB, secondaryX, secondaryY, secondaryRB, secondaryLB;
    private Trigger secondaryUp, secondaryDown, secondaryLeft, secondaryRight, secondaryLT, secondaryRT;

    private final SendableChooser<Command> auto_chooser;

    public RobotContainer() {
        PhotonVision.getInstance();
        Pivot.getInstance();
        Swerve.getInstance();
        auto_chooser = AutoBuilder.buildAutoChooser("Left Shoot Auto");

        // Button Initialization
        {
            mainController = new XboxController(0);
            mainRB = new JoystickButton(mainController, XboxController.Button.kRightBumper.value);
            mainLB = new JoystickButton(mainController, XboxController.Button.kLeftBumper.value);
            mainA = new JoystickButton(mainController, XboxController.Button.kA.value);
            mainB = new JoystickButton(mainController, XboxController.Button.kB.value);
            mainX = new JoystickButton(mainController, XboxController.Button.kX.value);
            mainY = new JoystickButton(mainController, XboxController.Button.kY.value);
            mainUp = new Trigger(() -> mainController.getPOV() == 0);
            mainRight = new Trigger(() -> mainController.getPOV() == 90);
            mainDown = new Trigger(() -> mainController.getPOV() == 180);
            mainLeft = new Trigger(() -> mainController.getPOV() == 270);
            mainRT = new Trigger(() -> mainController.getRightTriggerAxis() > 0.1);
            mainLT = new Trigger(() -> mainController.getLeftTriggerAxis() > 0.1);

            secondaryController = new XboxController(1);
            secondaryRB = new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value);
            secondaryLB = new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value);
            secondaryA = new JoystickButton(secondaryController, XboxController.Button.kA.value);
            secondaryB = new JoystickButton(secondaryController, XboxController.Button.kB.value);
            secondaryX = new JoystickButton(secondaryController, XboxController.Button.kX.value);
            secondaryY = new JoystickButton(secondaryController, XboxController.Button.kY.value);
            secondaryUp = new Trigger(() -> secondaryController.getPOV() == 0);
            secondaryRight = new Trigger(() -> secondaryController.getPOV() == 90);
            secondaryDown = new Trigger(() -> secondaryController.getPOV() == 180);
            secondaryLeft = new Trigger(() -> secondaryController.getPOV() == 270);
            secondaryRT = new Trigger(() -> secondaryController.getRightTriggerAxis() > 0.1);
            secondaryLT = new Trigger(() -> secondaryController.getLeftTriggerAxis() > 0.1);
        }

        // Named Commands
        {
            // Shooter
            NamedCommands.registerCommand("RampShooter", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(Constants.Shooter.CloseShootSpeeds.mainShooterRPS, Constants.Shooter.CloseShootSpeeds.topShaftRPS, -3);
            })); // TODO: MAYBE MAKE A SEPARATE CONSTANT FOR AUTO SHOOTING

            NamedCommands.registerCommand("StartShoot", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(Constants.Shooter.CloseShootSpeeds.mainShooterRPS, Constants.Shooter.CloseShootSpeeds.topShaftRPS, Constants.Shooter.CloseShootSpeeds.indexerRPS);
                Transfer.getInstance().setTransferSpeed(Constants.Transfer.transferSpeed);
            })); // TODO: MAYBE MAKE A SEPARATE CONSTANT FOR AUTO SHOOTING

            NamedCommands.registerCommand("StopShoot", new InstantCommand(() -> {
                Shooter.getInstance().Stop();
                Transfer.getInstance().setTransferSpeed(0);
            }));

            // Intake
            NamedCommands.registerCommand("ExtendIntake", new InstantCommand(() -> {
                Pivot.getInstance().setGoal(Constants.Pivot.downPos);
            }));
            NamedCommands.registerCommand("RetractIntake", new InstantCommand(() -> {
                Pivot.getInstance().setGoal(Constants.Pivot.upPos);
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
                () -> secondaryX.getAsBoolean()
        ));

        configureBindings();

    }

    private void configureBindings () {
        // Main Controller Binds
        {
            // Shoot
            mainRB.onTrue(new InstantCommand(() -> {
                Shooter.getInstance().Shoot();
                Transfer.getInstance().setTransferSpeed(Constants.Transfer.transferSpeed);
            })).onFalse(new InstantCommand(() -> {
                Shooter.getInstance().Stop();
                Transfer.getInstance().setTransferSpeed(0);
            }));

            // Zero Heading
            mainLB.onTrue(new InstantCommand(() -> {
                Swerve.getInstance().zeroHeading();
            }));
        }

        // Secondary Controller Binds
        {
            // Intake
            secondaryLB.onTrue(new InstantCommand(() -> {
                Intake.getInstance().setIntakeSpeed(Constants.Intake.intakeSpeedRPS);
                Transfer.getInstance().setTransferSpeed(Constants.Transfer.transferSpeed);

            })).onFalse(new InstantCommand(() -> {
                Intake.getInstance().cancelPID();
                Transfer.getInstance().setTransferSpeed(0);

            }));


            // Ramp Shooter
            secondaryRB.onTrue(new InstantCommand(() -> {
                Shooter.getInstance().Ramp();
            }));

            // PID Retract Intake
            secondaryA.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().cancelPID();
                Pivot.getInstance().setGoal(Constants.Pivot.upPos);
            })).onFalse(new InstantCommand(() -> {
                Pivot.getInstance().setSpeed(0);
                Pivot.getInstance().cancelPID();
            }));

            // PID Extend Intake
            secondaryB.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().cancelPID();
                Pivot.getInstance().setGoal(Constants.Pivot.downPos);
            })).onFalse(new InstantCommand(() -> {
                Pivot.getInstance().setSpeed(0);
                Pivot.getInstance().cancelPID();
            }));

            // Zero Pivot PID
            secondaryLT.onTrue(new InstantCommand(() -> Pivot.getInstance().resetAngle()));

            // Manual Extend Intake
            secondaryDown.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().setSpeed(.2);
            })).onFalse(new InstantCommand(() -> {
                Pivot.getInstance().setSpeed(0);
            }));

            // Manual Retract Intake
            secondaryUp.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().setSpeed(-.2);
            })).onFalse(new InstantCommand(() -> {
                Pivot.getInstance().setSpeed(0);
            }));

            // Toggle Shooting Distance
            secondaryY.onTrue(new InstantCommand(() -> Shooter.getInstance().cycleSpeeds()));
        }
    }


    public Command getAutonomousCommand () {
        return auto_chooser.getSelected();
    }
}

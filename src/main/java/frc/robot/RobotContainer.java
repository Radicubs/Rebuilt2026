// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.teleop.TeleopSwerve;
import frc.robot.subsystems.*;


public class RobotContainer {

    XboxController mainController;
    private JoystickButton mainA, mainB, mainX, mainY, mainRB, mainLB;
    private Trigger mainUp, mainDown, mainLeft, mainRight, mainLT, mainRT, mainBack;

    XboxController secondaryController;
    private JoystickButton secondaryA, secondaryB, secondaryX, secondaryY, secondaryRB, secondaryLB;
    private Trigger secondaryUp, secondaryDown, secondaryLeft, secondaryRight, secondaryLT, secondaryRT, secondaryBack;

    private final SendableChooser<Command> auto_chooser = new SendableChooser<Command>();

    public RobotContainer() {
        PhotonVision.getInstance();
        Pivot.getInstance();
        Swerve.getInstance();
        Shooter.getInstance();
        Transfer.getInstance();

        // Named Commands
        {
            // Shooter
            NamedCommands.registerCommand("RampShooter", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(Constants.Shooter.CloseShootSpeeds.mainShooterRPS, Constants.Shooter.CloseShootSpeeds.topShaftRPS, -3);
            }));

            NamedCommands.registerCommand("StartShoot", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(Constants.Shooter.CloseShootSpeeds.mainShooterRPS, Constants.Shooter.CloseShootSpeeds.topShaftRPS, Constants.Shooter.CloseShootSpeeds.indexerRPS);
                Transfer.getInstance().setTransferSpeed(Constants.Transfer.transferSpeed);
            }));

            NamedCommands.registerCommand("RampTrenchShooter", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(Constants.Shooter.TrenchShootSpeeds.mainShooterRPS, Constants.Shooter.TrenchShootSpeeds.topShaftRPS, -3);
            }));

            NamedCommands.registerCommand("StartTrenchShooter", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(Constants.Shooter.TrenchShootSpeeds.mainShooterRPS, Constants.Shooter.TrenchShootSpeeds.topShaftRPS, Constants.Shooter.TrenchShootSpeeds.indexerRPS);
                Transfer.getInstance().setTransferSpeed(Constants.Transfer.transferSpeed);
            }));

            NamedCommands.registerCommand("StopShoot", new InstantCommand(() -> {
                Shooter.getInstance().Stop();
                Transfer.getInstance().setTransferSpeed(0);
            }));

            // Intake
            NamedCommands.registerCommand("ExtendIntake", new InstantCommand(() -> {
                Pivot.getInstance().setGoal(Constants.Pivot.downPos);
            }));
            NamedCommands.registerCommand("RetractIntake", new InstantCommand(() -> {
                Pivot.getInstance().setSpeed(0);
                Pivot.getInstance().setGoal(Constants.Pivot.upPos);
            }));

            NamedCommands.registerCommand("StartIntake", new InstantCommand(() -> {
                Intake.getInstance().setIntakeSpeed(Constants.Intake.intakeSpeedRPS);
                Pivot.getInstance().setSpeed(-0.1);
            }));

            NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {
                Intake.getInstance().setIntakeSpeed(0);
            }));
        }

        // Auto Chooser
        {
            try{
                auto_chooser.setDefaultOption("Left Shoot", AutoBuilder.buildAuto("Left Shoot Auto"));
                auto_chooser.addOption("Right Shoot", AutoBuilder.buildAuto("Right Shoot Auto"));
                auto_chooser.addOption("Middle Shoot", AutoBuilder.buildAuto("Middle Shoot Auto"));
                auto_chooser.addOption("Left to Depot", AutoBuilder.buildAuto("Left Depot Auto"));
                auto_chooser.addOption("Middle to Depot", AutoBuilder.buildAuto("Middle Depot Auto"));
                auto_chooser.addOption("Right to Outpost", AutoBuilder.buildAuto("Right Outpost Auto"));
                auto_chooser.addOption("Middle to Outpost", AutoBuilder.buildAuto("Middle Outpost Auto"));
                auto_chooser.addOption("Left to Center", AutoBuilder.buildAuto("Left Center Auto "));
                auto_chooser.addOption("Right to Center", AutoBuilder.buildAuto("Right Center Auto"));
            }
            catch(Exception e) {
                System.out.println("Error" + e.getMessage());
                auto_chooser.setDefaultOption("Auto Error", new InstantCommand());
            }

            SmartDashboard.putData("Auto Chooser", auto_chooser);
        }

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
            mainBack = new Trigger(() -> mainController.getBackButton());

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
            secondaryBack = new Trigger(() -> secondaryController.getBackButton());
        }


        Swerve.getInstance().setDefaultCommand(new TeleopSwerve(
                () -> -mainController.getLeftY(),
                () -> -mainController.getLeftX(),
                () -> -mainController.getRightX(),
                () -> mainX.getAsBoolean()
        ));

        configureBindings();

    }

    private void configureBindings () {
        // Main Controller Binds
        {
            // Shoot
            mainRB.onTrue(new InstantCommand(() -> {
                Shooter.getInstance().setIndexerSpeed(Constants.Shooter.CloseShootSpeeds.indexerRPS);
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


            // Close Ramp Shooter
            secondaryRB.onTrue(new InstantCommand(() -> {
                Shooter.getInstance().CloseRamp();
            })).onFalse(new InstantCommand(()->{
                Shooter.getInstance().cancelPID();
            }));

            // Trench Ramp
            secondaryRT.onTrue(new InstantCommand(() -> {
                Shooter.getInstance().TrenchRamp();
            })).onFalse(new InstantCommand(()->{
                Shooter.getInstance().cancelPID();
            }));

            // PID Retract Intake
            secondaryB.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().cancelPID();
                Pivot.getInstance().setGoal(Constants.Pivot.upPos);
            })).onFalse(new InstantCommand(() -> {
                Pivot.getInstance().setSpeed(0);
            }));

            // PID Extend Intake
            secondaryA.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().cancelPID();
                Pivot.getInstance().setGoal(Constants.Pivot.downPos);
            })).onFalse(new InstantCommand(() -> {
                Pivot.getInstance().setSpeed(0);
            }));

            // Zero Pivot PID
            secondaryBack.onTrue(new InstantCommand(() -> Pivot.getInstance().resetAngle()));

            // Manual Extend Intake
            secondaryDown.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().cancelPID();
                Pivot.getInstance().setSpeed(.2);
            })).onFalse(new InstantCommand(() -> {
                Pivot.getInstance().setSpeed(0);
                Intake.getInstance().setIntakeSpeed(0);
                Intake.getInstance().cancelPID();
            }));

            // Manual Retract Intake
            secondaryUp.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().cancelPID();
                Pivot.getInstance().setSpeed(-.2);
                Intake.getInstance().setIntakeSpeed(Constants.Intake.intakeSpeedRPS);
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

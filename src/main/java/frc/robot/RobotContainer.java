// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.intake.*;
import frc.robot.commands.pivot.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.transfer.*;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.constants.TransferConstants;
import frc.robot.util.HubState;

import java.lang.constant.Constable;


public class RobotContainer {

    CommandXboxController mainController;
    CommandXboxController secondaryController;
    // Subsystems
    Vision vision = Vision.getInstance();
    Pivot pivot = Pivot.getInstance();
    Drive drive = Drive.getInstance();
    Shooter shooter = Shooter.getInstance();
    Transfer transfer = Transfer.getInstance();
    Intake intake = Intake.getInstance();
    private final SendableChooser<Command> auto_chooser = new SendableChooser<Command>();

    public RobotContainer() {
        registerNamedCommands();   // must precede the auto chooser — AutoBuilder resolves these names
        configureAutoChooser();
        configureDashboard();
        configureControllers();
        configureBindings();
    }

    private void registerNamedCommands() {
        // ---- Shooter / transfer ----
        // Ramp the flywheels (indexer held back at -3, no belt).
//        NamedCommands.registerCommand("Ramp Close Shot",
//                new SetShooterSpeeds(shooter, ShooterConstants.CloseShootSpeeds.mainShooterRPS, ShooterConstants.CloseShootSpeeds.topShaftRPS, -3));

        NamedCommands.registerCommand("Ramp Trench Shot",
                new SetShooterSpeeds(shooter, ShooterConstants.TrenchShootSpeeds.mainShooterRPS, ShooterConstants.TrenchShootSpeeds.topShaftRPS, -3));

        // Full shot: flywheels + indexer + belt.
//        NamedCommands.registerCommand("Start Close Shot",
//                new SetShooterSpeeds(shooter, ShooterConstants.CloseShootSpeeds.mainShooterRPS, ShooterConstants.CloseShootSpeeds.topShaftRPS, ShooterConstants.CloseShootSpeeds.indexerRPS)
//                        .alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed))
//                        .withTimeout(6.0));

        NamedCommands.registerCommand("Start Trench Shot",
                new SetShooterSpeeds(shooter, ShooterConstants.TrenchShootSpeeds.mainShooterRPS, ShooterConstants.TrenchShootSpeeds.topShaftRPS, ShooterConstants.TrenchShootSpeeds.indexerRPS)
                        .alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed))
                        .withTimeout(6.0));

        // Reverse/eject (shooter only, no belt).
        NamedCommands.registerCommand("Eject",
                new SetShooterSpeeds(shooter, ShooterConstants.EjectSpeeds.mainShooterRPS, ShooterConstants.EjectSpeeds.topShaftRPS, ShooterConstants.EjectSpeeds.indexerRPS)
                        .withTimeout(1));

        // ---- Intake / pivot ----
        NamedCommands.registerCommand("Start Intake",
                new SetIntakeSpeed(intake, IntakeConstants.intakeSpeedRPS)
                        .alongWith(new SetPivotSpeed(pivot, 0.07)));

        NamedCommands.registerCommand("Stop Intake", new SetIntakeSpeed(intake, 0));

        NamedCommands.registerCommand("Extend Pivot", new SetPivotPosition(pivot, PivotConstants.downPos));

        NamedCommands.registerCommand("Retract Pivot", new SetPivotPosition(pivot, PivotConstants.upPos));

        NamedCommands.registerCommand("Shake Pivot", new ShakePivot(pivot));

        NamedCommands.registerCommand("Reset Heading", new SetHeading(drive, () -> drive.getHeading().plus(Rotation2d.k180deg)));
    }

    private void configureAutoChooser() {
        try {
            auto_chooser.setDefaultOption("Left Shoot", AutoBuilder.buildAuto("Left Shoot Auto"));
            auto_chooser.addOption("Right Shoot", AutoBuilder.buildAuto("Right Shoot Auto"));
            auto_chooser.addOption("Middle Shoot", AutoBuilder.buildAuto("Middle Shoot Auto"));
            auto_chooser.addOption("Left Center Cycle", AutoBuilder.buildAuto("Left Center Cycle Auto"));
            auto_chooser.addOption("Right Center Cycle", AutoBuilder.buildAuto("Right Center Cycle Auto"));
            auto_chooser.addOption("Left Center Cycle Long", AutoBuilder.buildAuto("Left Center Cycle Long Auto"));
            auto_chooser.addOption("Right Center Cycle Long", AutoBuilder.buildAuto("Right Center Cycle Long Auto"));
            auto_chooser.addOption("Middle Depot", AutoBuilder.buildAuto("Middle Depot Auto"));
        } catch (Exception e) {
            System.out.println("Error" + e.getMessage());
            auto_chooser.setDefaultOption("Auto Error", new InstantCommand());
        }
        SmartDashboard.putData("Auto Chooser", auto_chooser);
    }

    private void configureDashboard() {
        SmartDashboard.putData("Field Elements", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Hub Active", HubState::isActive, null);
            }
        });

            SmartDashboard.putData("Distance From Hub", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Distance", () -> FieldConstants.hubCenterBlue.getTranslation().getDistance(drive.getPose().getTranslation()), null);
            }
        });
    }

    private void configureControllers() {
        mainController = new CommandXboxController(0);
//        secondaryController = new CommandXboxController(1);

        drive.setDefaultCommand(new TeleopDrive(
                () -> -mainController.getLeftY(),
                () -> -mainController.getLeftX(),
                () -> -mainController.getRightX(),
                () -> false //mainController.x().getAsBoolean()
        ));
    }

    private void configureBindings () {

        // Main Controller Binds
        {
            //Shoot -> Run Indexer + Transfer
            mainController.rightBumper().whileTrue(
                    new SetShooterSpeeds(shooter, 25,15,ShooterConstants.CloseShootSpeeds.indexerRPS)
                    .alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed)));

            mainController.x().whileTrue(
                    new SetIntakeSpeed(intake, IntakeConstants.intakeSpeedRPS)
                            .alongWith(new SetPivotSpeed(pivot, 0.07))
                            .onlyIf(() -> Math.abs(pivot.getSpeed()) < 0.05));

            mainController.povUp().onTrue(
                    new SetPivotPosition(pivot,PivotConstants.upPos));

            mainController.povDown().onTrue(
                    new SetPivotPosition(pivot,PivotConstants.downPos));



//            mainController.leftBumper().onTrue(new ZeroHeading(drive));
//
//            // Flip Heading
//            mainController.//            // Zero HeadingpovDown().onTrue(new SetHeading(drive, Rotation2d.k180deg));
        }

        // Secondary Controller Binds
        {
            // Intake (only if the pivot isn't already moving)



////            // Close Ramp
////            secondaryController.rightBumper().whileTrue(
////                    new SetShooterSpeeds(shooter, ShooterConstants.CloseShootSpeeds.mainShooterRPS, ShooterConstants.CloseShootSpeeds.topShaftRPS,-3));
////
////            // Trench Ramp
////            secondaryController.rightTrigger().whileTrue(
////                    new SetShooterSpeeds(shooter, ShooterConstants.TrenchShootSpeeds.mainShooterRPS, ShooterConstants.TrenchShootSpeeds.topShaftRPS,-3));
////
////            // Pass Ramp
////            secondaryController.leftTrigger().whileTrue(
////                    new SetShooterSpeeds(shooter, ShooterConstants.PassSpeeds.mainShooterRPS, ShooterConstants.PassSpeeds.topShaftRPS,-3));
////
////            // PID Retract Intake
////            secondaryController.b().onTrue(new SetPivotPosition(pivot, PivotConstants.upPos));
////
////            // PID Extend Intake
////            secondaryController.a().onTrue(new SetPivotPosition(pivot, PivotConstants.downPos));
//
//            // Zero Pivot PID
//            secondaryController.back().onTrue(new ResetPivotAngle(pivot));

            // Manual Extend Intake
//            new Trigger(() -> Math.abs(mainController.getLeftY()) > 0.1).whileTrue(
//                    new SetPivotSpeed(pivot, () -> secondaryController.getLeftY() * 0.2)
//            );

            // Shift Main Shooter Up

//
//            // Shift Top Shooter Up
//            secondaryController.povLeft().onTrue(
//                    new AdjustCustomShot(shooter, 0.0,0.5));
//
//            // Shift Top Shooter Down
//            secondaryController.povRight().onTrue(
//                    new AdjustCustomShot(shooter, 0.0,-0.5));
//
//            // Regression Shooting
//            secondaryController.y().whileTrue(
//                    new SetShooterSpeeds(shooter, shooter::getRegressionMainSpeed, shooter::getRegressionTopSpeed, () -> ShooterConstants.CloseShootSpeeds.indexerRPS));

        }
    }

    public Command getAutonomousCommand () {
        return auto_chooser.getSelected();
    }
}

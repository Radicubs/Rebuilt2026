// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.*;
import frc.robot.commands.pivot.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.swerve.*;
import frc.robot.commands.transfer.*;
import frc.robot.subsystems.field.FieldConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferConstants;

import java.util.Optional;


public class RobotContainer {

    CommandXboxController mainController;
    CommandXboxController secondaryController;
    // Subsystems
    PhotonVision photonvision = PhotonVision.getInstance();
    Pivot pivot = Pivot.getInstance();
    Swerve swerve = Swerve.getInstance();
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

    /** Registers the PathPlanner NamedCommands referenced by the autos / path event markers. */
    private void registerNamedCommands() {
        // ---- Shooter / transfer ----
        // Ramp the flywheels (indexer held back at -3, no belt).
        NamedCommands.registerCommand("Ramp Close Shot",
                new SetShooterSpeeds(shooter, ShooterConstants.CloseShootSpeeds.mainShooterRPS, ShooterConstants.CloseShootSpeeds.topShaftRPS, -3));

        NamedCommands.registerCommand("Ramp Trench Shot",
                new SetShooterSpeeds(shooter, ShooterConstants.TrenchShootSpeeds.mainShooterRPS, ShooterConstants.TrenchShootSpeeds.topShaftRPS, -3));

        // Full shot: flywheels + indexer + belt.
        NamedCommands.registerCommand("Start Close Shot",
                new SetShooterSpeeds(shooter, ShooterConstants.CloseShootSpeeds.mainShooterRPS, ShooterConstants.CloseShootSpeeds.topShaftRPS, ShooterConstants.CloseShootSpeeds.indexerRPS)
                        .alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed))
                        .withTimeout(6.0));

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

        NamedCommands.registerCommand("Reset Heading", new SetHeading(swerve, () -> swerve.getHeading().plus(Rotation2d.k180deg)));
    }

    /** Builds the autonomous chooser from the PathPlanner autos. */
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

    /** Publishes match / field telemetry to the dashboard. */
    private void configureDashboard() {
        SmartDashboard.putData("Field Elements", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Hub Active", () -> isHubActive(), null);
            }
        });

            SmartDashboard.putData("Distance From Hub", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Distance", () -> FieldConstants.hubCenterBlue.getTranslation().getDistance(swerve.getPose().getTranslation()), null);
            }
        });
    }

    /** Creates the driver controllers and the swerve default (teleop drive) command. */
    private void configureControllers() {
        mainController = new CommandXboxController(0);
        secondaryController = new CommandXboxController(1);

        swerve.setDefaultCommand(new TeleopSwerve(
                () -> -mainController.getLeftY(),
                () -> -mainController.getLeftX(),
                () -> -mainController.getRightX(),
                () -> mainController.x().getAsBoolean()
        ));
    }

    private void configureBindings () {

        // Main Controller Binds
        {
            //Shoot -> Run Indexer + Transfer
            mainController.rightBumper().whileTrue(
                    new SetShooterSpeeds(shooter, 0,0,ShooterConstants.CloseShootSpeeds.indexerRPS)
                    .alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed)));
            
            // Zero Heading
            mainController.leftBumper().onTrue(new ZeroHeading(swerve));

            // Flip Heading
            mainController.povDown().onTrue(new SetHeading(swerve, Rotation2d.k180deg));
        }

        // Secondary Controller Binds
        {
            // Intake (only if the pivot isn't already moving)
            secondaryController.x().whileTrue(
                    new SetIntakeSpeed(intake, IntakeConstants.intakeSpeedRPS)
                            .alongWith(new SetPivotSpeed(pivot, 0.07))
                            .onlyIf(() -> Math.abs(pivot.getSpeed()) < 0.05));


            // Close Ramp
            secondaryController.rightBumper().whileTrue(
                    new SetShooterSpeeds(shooter, ShooterConstants.CloseShootSpeeds.mainShooterRPS, ShooterConstants.CloseShootSpeeds.topShaftRPS,-3));

            // Trench Ramp
            secondaryController.rightTrigger().whileTrue(
                    new SetShooterSpeeds(shooter, ShooterConstants.TrenchShootSpeeds.mainShooterRPS, ShooterConstants.TrenchShootSpeeds.topShaftRPS,-3));

            // Pass Ramp
            secondaryController.leftTrigger().whileTrue(
                    new SetShooterSpeeds(shooter, ShooterConstants.PassSpeeds.mainShooterRPS, ShooterConstants.PassSpeeds.topShaftRPS,-3));

            // PID Retract Intake
            secondaryController.b().onTrue(new SetPivotPosition(pivot, PivotConstants.upPos));

            // PID Extend Intake
            secondaryController.a().onTrue(new SetPivotPosition(pivot, PivotConstants.downPos));

            // Zero Pivot PID
            secondaryController.back().onTrue(new ResetPivotAngle(pivot));

            // Manual Extend Intake
            secondaryController.leftStick().whileTrue(
                    new SetPivotSpeed(pivot, () -> secondaryController.getLeftY() * 0.2));

            // Shift Main Shooter Up
            secondaryController.povUp().onTrue(
                    new AdjustCustomShot(shooter, 0.5,0.0));

            // Shift Main Shooter Down
            secondaryController.povDown().onTrue(
                    new AdjustCustomShot(shooter, -0.5,0.0));

            // Shift Top Shooter Up
            secondaryController.povLeft().onTrue(
                    new AdjustCustomShot(shooter, 0.0,0.5));

            // Shift Top Shooter Down
            secondaryController.povRight().onTrue(
                    new AdjustCustomShot(shooter, 0.0,-0.5));

            // Regression Shooting
            secondaryController.y().whileTrue(
                    new SetShooterSpeeds(shooter, shooter::getRegressionMainSpeed, shooter::getRegressionTopSpeed, () -> ShooterConstants.CloseShootSpeeds.indexerRPS));

        }
    }

    public Command getAutonomousCommand () {
        return auto_chooser.getSelected();
    }

    public boolean isHubActive() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}

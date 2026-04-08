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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.teleop.TeleopSwerve;
import frc.robot.subsystems.*;

import java.util.Optional;


public class RobotContainer {

    XboxController mainController;
    private JoystickButton mainA, mainB, mainX, mainY, mainRB, mainLB;
    private Trigger mainUp, mainDown, mainLeft, mainRight, mainLT, mainRT, mainBack;

    XboxController secondaryController;
    private JoystickButton secondaryA, secondaryB, secondaryX, secondaryY, secondaryRB, secondaryLB;
    private Trigger secondaryUp, secondaryDown, secondaryLeft, secondaryRight, secondaryLT, secondaryRT, secondaryBack, secondaryStickUp, secondaryStickDown, secondaryStart;

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
                Transfer.getInstance().setTransferSpeed(Constants.Transfer.shootTransferSpeed);
            }));

            NamedCommands.registerCommand("RampTrenchShooter", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(Constants.Shooter.TrenchShootSpeeds.mainShooterRPS, Constants.Shooter.TrenchShootSpeeds.topShaftRPS, -3);
            }));

            NamedCommands.registerCommand("StartTrenchShooter", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(Constants.Shooter.TrenchShootSpeeds.mainShooterRPS, Constants.Shooter.TrenchShootSpeeds.topShaftRPS, Constants.Shooter.TrenchShootSpeeds.indexerRPS);
                Transfer.getInstance().setTransferSpeed(Constants.Transfer.shootTransferSpeed);
            }));

            NamedCommands.registerCommand("StopShoot", new InstantCommand(() -> {
                Shooter.getInstance().stop();
                Transfer.getInstance().setTransferSpeed(0);
            }));

            NamedCommands.registerCommand("ReverseShoot", new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(-10, -10, -10);
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
                Pivot.getInstance().setSpeed(0.07);
            }));

            NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {
                Intake.getInstance().setIntakeSpeed(0);
            }));

            NamedCommands.registerCommand("Reset Heading", new InstantCommand(() -> Swerve.getInstance().setHeading(Swerve.getInstance().getHeading().plus(Rotation2d.k180deg))));
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
                auto_chooser.addOption("Left to Center Rush", AutoBuilder.buildAuto("Left Center Rush Auto"));
                auto_chooser.addOption("Right to Center Rush", AutoBuilder.buildAuto("Right Center Rush Auto"));
                auto_chooser.addOption("Left to Center Cycle", AutoBuilder.buildAuto("Left Center Cycle Auto"));
                auto_chooser.addOption("Right to Center Cycle", AutoBuilder.buildAuto("Right Center Cycle Auto"));
                auto_chooser.addOption("Left to Center Stay", AutoBuilder.buildAuto("Left Center Stay Auto"));
                auto_chooser.addOption("Right to Center Stay", AutoBuilder.buildAuto("Right Center Stay Auto"));
                auto_chooser.addOption("Left Center Cycle Straight", AutoBuilder.buildAuto("Left Center Cycle Straight"));
                auto_chooser.addOption("Right Center Cycle Straight", AutoBuilder.buildAuto("Right Center Cycle Straight"));
            }
            catch(Exception e) {
                System.out.println("Error" + e.getMessage());
                auto_chooser.setDefaultOption("Auto Error", new InstantCommand());
            }

            SmartDashboard.putData("Auto Chooser", auto_chooser);
            SmartDashboard.putData("Field Elements", new Sendable() {
                @Override
                public void initSendable(SendableBuilder builder) {
                    builder.addBooleanProperty("Hub Active", () -> isHubActive(), null);
                }
            });

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
            secondaryStickUp = new Trigger(() -> secondaryController.getLeftY() < -0.5);
            secondaryStickDown = new Trigger(() -> secondaryController.getLeftY() > 0.5);
            secondaryStart = new Trigger(() -> secondaryController.getStartButton());

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
                Transfer.getInstance().setTransferSpeed(Constants.Transfer.shootTransferSpeed);
            })).onFalse(new InstantCommand(() -> {
                Transfer.getInstance().setTransferSpeed(0);
                Shooter.getInstance().setIndexerSpeed(0);
            }));

            // Zero Heading
            mainLB.onTrue(new InstantCommand(() -> {
                Swerve.getInstance().zeroHeading();
            }));

            mainDown.onTrue(new InstantCommand(() -> Swerve.getInstance().setHeading(Rotation2d.k180deg)));
        }

        // Secondary Controller Binds
        {
            // Intake
            secondaryX.onTrue(new InstantCommand(() -> {
                if(Math.abs(Pivot.getInstance().getSpeed()) < 0.05){
                    Intake.getInstance().setIntakeSpeed(Constants.Intake.intakeSpeedRPS);
//                    Transfer.getInstance().setTransferSpeed(Constants.Transfer.intakeTransferSpeed);
                    Pivot.getInstance().setSpeed(0.07);
                }

            })).onFalse(new InstantCommand(() -> {
                Intake.getInstance().cancelPID();
                Transfer.getInstance().setTransferSpeed(0);
                Pivot.getInstance().setSpeed(0);
            }));


            // Close Ramp
            secondaryRB.onTrue(new InstantCommand(() -> Shooter.getInstance().closeRamp()))
                    .onFalse(new InstantCommand(()-> Shooter.getInstance().cancelPID()));

            // Trench Ramp
            secondaryRT.onTrue(new InstantCommand(() -> Shooter.getInstance().trenchRamp()))
                    .onFalse(new InstantCommand(()->Shooter.getInstance().cancelPID()));

            // Pass Ramp
            secondaryLT.onTrue(new InstantCommand(() -> Shooter.getInstance().passRamp()))
                    .onFalse(new InstantCommand(() -> Shooter.getInstance().cancelPID()));

            // Custom Ramp
            secondaryLB.onTrue(new InstantCommand(() -> {
                Shooter.getInstance().customRamp();
            })).onFalse(new InstantCommand(() -> Shooter.getInstance().cancelPID()));

            // PID Retract Intake
            secondaryB.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().cancelPID();
                Pivot.getInstance().setGoal(Constants.Pivot.middlePos);
            })).onFalse(new InstantCommand(() -> Pivot.getInstance().setSpeed(0)));

            // PID Extend Intake
            secondaryA.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().cancelPID();
                Pivot.getInstance().setGoal(Constants.Pivot.downPos);
            })).onFalse(new InstantCommand(() -> Pivot.getInstance().setSpeed(0)));

            // Zero Pivot PID
            secondaryBack.onTrue(new InstantCommand(() -> Pivot.getInstance().resetAngle()));

            // Manual Extend Intake
            secondaryStickDown.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().cancelPID();
                Pivot.getInstance().setSpeed(.2);
            })).onFalse(new InstantCommand(() -> Pivot.getInstance().setSpeed(0)));

            // Manual Retract Intake
            secondaryStickUp.onTrue(new InstantCommand(() -> {
                Pivot.getInstance().cancelPID();
                Pivot.getInstance().setSpeed(-.2);
                Intake.getInstance().setIntakeSpeed(30);
            })).onFalse(new InstantCommand(() -> {
                Pivot.getInstance().setSpeed(0);
                Intake.getInstance().setIntakeSpeed(0);
            }));

            // Manual Shoot Shift Up
            secondaryUp.onTrue(new InstantCommand(() -> {
                Shooter.getInstance().changeShooterSpeeds(.5);
            }))
                    .onFalse(new InstantCommand(() -> Shooter.getInstance().changeShooterSpeeds(0)));

            // Manual Shoot Shift Down
            secondaryDown.onTrue(new InstantCommand(() -> Shooter.getInstance().changeShooterSpeeds(-.5)))
                    .onFalse(new InstantCommand(() -> Shooter.getInstance().changeShooterSpeeds(0)));

            // Spit Shooter
            secondaryLeft.onTrue(new InstantCommand(() -> {
                Shooter.getInstance().setShooterSpeeds(-10, -2, -7);
                Transfer.getInstance().setTransferSetpoint(-10);
            }))
                .onFalse(new InstantCommand(() -> {
                    Shooter.getInstance().stop();
                    Transfer.getInstance().setTransferSetpoint(0);
                }));

//            secondaryLeft.onTrue(new InstantCommand(() -> Shooter.getInstance().changeIndexerSpeeds(.5)));
//            secondaryRight.onTrue(new InstantCommand(() -> Shooter.getInstance().changeIndexerSpeeds(-.5)));

            // Spit Intake
            secondaryRight.onTrue(new InstantCommand(() -> Intake.getInstance().setIntakeSpeed(-15)))
                    .onFalse(new InstantCommand(() -> Intake.getInstance().setIntakeSpeed(0)));

            secondaryStart.onTrue(new InstantCommand(() -> {
                //Shooter.getInstance().setIndexerSpeed(Constants.Shooter.CloseShootSpeeds.indexerRPS);
                Transfer.getInstance().setTransferSpeed(1);//Constants.Transfer.shootTransferSpeed
            })).onFalse(new InstantCommand(() -> {
                Transfer.getInstance().setTransferSpeed(0);
            }));

            // Regression Shooting
            secondaryY.whileTrue(Commands.runEnd(() -> Shooter.getInstance().regressionRamp(), () -> Shooter.getInstance().stop()));

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

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.teleop.TeleopSwerve;
import frc.robot.subsystems.Swerve;


public class RobotContainer {
    CommandXboxController mainController = new CommandXboxController(0);

    Trigger left, up, down, right;

    public RobotContainer() {


        mainController = new CommandXboxController(0);
        Swerve.getInstance().setDefaultCommand(new TeleopSwerve(
                () -> -mainController.getLeftY(),
                () -> -mainController.getLeftX(),
                () -> mainController.getRightX()
        ));

    }
    private void configureBindings () {
    }


    public Command getAutonomousCommand ()
    {
        return new InstantCommand();
    }
}

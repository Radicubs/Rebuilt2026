package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;

public class ZeroHeading extends InstantCommand {

    public ZeroHeading(Drive drive) {
        super(drive::zeroHeading);
    }
}

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;

import java.util.function.Supplier;

public class SetHeading extends InstantCommand {

    public SetHeading(Drive drive, Supplier<Rotation2d> heading) {
        super(() -> drive.setHeading(heading.get()));
    }

    public SetHeading(Drive drive, Rotation2d heading) {
        this(drive, () -> heading);
    }
}

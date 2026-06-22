package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.Supplier;

public class SetHeading extends InstantCommand {

    public SetHeading(Swerve swerve, Supplier<Rotation2d> heading) {
        super(() -> swerve.setHeading(heading.get()));
    }
    public SetHeading(Swerve swerve, Rotation2d heading) {
        this(swerve, () -> heading);
    }
}

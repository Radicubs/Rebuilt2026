package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.Swerve;
public class ZeroHeading extends InstantCommand {
    public ZeroHeading(Swerve swerve) {
        super(swerve::zeroHeading);
    }
}

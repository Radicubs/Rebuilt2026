package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.Shooter;

public class AdjustCustomShot extends InstantCommand {

    public AdjustCustomShot(Shooter shooter, double mainShooterChange, double topShooterChange) {
        //super(() -> shooter.adjustCustomSpeeds(mainShooterChange, topShooterChange), shooter);
    }
}

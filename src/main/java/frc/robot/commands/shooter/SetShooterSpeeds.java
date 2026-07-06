package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class SetShooterSpeeds extends Command {

    private final Shooter shooter;
    private final DoubleSupplier mainShooterRPS;
    private final DoubleSupplier topShooterRPS;
    private final DoubleSupplier indexerRPS;

    public SetShooterSpeeds(Shooter shooter, DoubleSupplier mainShooterRPS, DoubleSupplier topShooterRPS, DoubleSupplier indexerRPS) {
        this.shooter = shooter;
        this.mainShooterRPS = mainShooterRPS;
        this.topShooterRPS = topShooterRPS;
        this.indexerRPS = indexerRPS;
        addRequirements(shooter);
    }

    public SetShooterSpeeds(Shooter shooter, double mainShooterRPS, double topShooterRPS, double indexerRPS) {
        this(shooter, () -> mainShooterRPS, () -> topShooterRPS, () -> indexerRPS);
    }

    @Override
    public void execute() {
        shooter.setShooterSpeeds(mainShooterRPS.getAsDouble(), topShooterRPS.getAsDouble(), indexerRPS.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.cancelPID();
    }
}

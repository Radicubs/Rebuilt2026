package frc.robot.commands.transfer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;

import java.util.function.DoubleSupplier;

/** Runs the transfer belt to move notes up to the shooter until interrupted. */
public class SetTransferSpeed extends Command {

    private Transfer transfer;
    private DoubleSupplier transferRPS;

    public SetTransferSpeed(Transfer transfer, DoubleSupplier transferRPS) {
        this.transfer = transfer;
        this.transferRPS = transferRPS;
        addRequirements(transfer);
    }

    /** Convenience constructor for fixed speeds. */
    public SetTransferSpeed(Transfer transfer, double transferRPS) {
        this(transfer, () -> transferRPS);
    }

    @Override
    public void execute() {
        transfer.setTransferSpeed(transferRPS.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        transfer.cancelPID();
    }
}
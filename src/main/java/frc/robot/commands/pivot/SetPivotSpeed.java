package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

import java.util.function.DoubleSupplier;

public class SetPivotSpeed extends Command {

    private final Pivot pivot;
    private final DoubleSupplier speed;

    public SetPivotSpeed(Pivot pivot, DoubleSupplier speed) {
        this.pivot = pivot;
        this.speed = speed;
        addRequirements(pivot);
    }

    /** Convenience constructor for a fixed open-loop output. */
    public SetPivotSpeed(Pivot pivot, double speed) {
        this(pivot, () -> speed);
    }

    @Override
    public void execute() {
        pivot.setSpeed(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        pivot.cancelPID();
    }
}

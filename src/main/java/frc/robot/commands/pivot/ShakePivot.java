package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.constants.PivotConstants;

public class ShakePivot extends Command {

    private static final double SHAKE_SPEED = 0.2;
    private static final double TOGGLE_PERIOD = 0.5; // seconds between direction flips
    private static final double DURATION = 5.0;      // total shake time

    private final Pivot pivot;
    private final Timer timer = new Timer();

    public ShakePivot(Pivot pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        // Flip direction every TOGGLE_PERIOD, starting negative to match the original routine.
        boolean flipped = (((int) (timer.get() / TOGGLE_PERIOD)) % 2) == 1;
        pivot.setSpeed(flipped ? SHAKE_SPEED : -SHAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(DURATION);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setGoal(PivotConstants.downPos);
    }
}

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.pivot.Pivot;

import java.util.function.DoubleSupplier;

public class SetPivotPosition extends InstantCommand {

    public SetPivotPosition(Pivot pivot, double position) {
        super(() -> pivot.setGoal(position), pivot);
    }

}

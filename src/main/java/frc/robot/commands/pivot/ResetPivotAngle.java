package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.pivot.Pivot;

public class ResetPivotAngle extends InstantCommand {

    public ResetPivotAngle(Pivot pivot) {
        super(pivot::resetAngle, pivot);
    }
}

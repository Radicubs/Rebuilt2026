package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.transfer.Transfer;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class SetIntakeSpeed extends Command {

    private Intake intake;
    private DoubleSupplier intakeRPS;

    public SetIntakeSpeed(Intake intake, DoubleSupplier intakeRPS) {
        this.intake = intake;
        this.intakeRPS = intakeRPS;
        addRequirements(intake);
    }

    /** Convenience constructor for fixed speeds. */
    public SetIntakeSpeed(Intake intake, double intakeRPS) {
        this(intake, () -> intakeRPS);
    }

    @Override
    public void execute() {
        intake.setVelocity(intakeRPS.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.cancelPID();
    }
}
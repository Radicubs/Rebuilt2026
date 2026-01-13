package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
    DoubleSupplier translationX;
    DoubleSupplier translationY;
    DoubleSupplier rotation;
    private Swerve swerve;
    public TeleopSwerve(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;
        swerve = Swerve.getInstance();
        addRequirements(swerve);

    }

    @Override
    public void execute() {
        swerve.drive(
                new Translation2d(
                        translationX.getAsDouble() * Constants.Swerve.maxSpeed,
                        translationY.getAsDouble() * Constants.Swerve.maxSpeed
                ),
                rotation.getAsDouble() * Constants.Swerve.maxAngularVelocity,
                true,
                false);
    }
    public void initialize() {swerve.resetModulesToAbsolute();}
}
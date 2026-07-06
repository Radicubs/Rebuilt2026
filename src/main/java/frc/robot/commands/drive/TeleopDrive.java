package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {

    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;
    private final DoubleSupplier rotation;
    private final BooleanSupplier toggleLockOn;

    private final Drive drive;
    private final PIDController lockOnPID;

    private boolean lockOn;
    private boolean prevToggle;

    public TeleopDrive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier toggleLockOn) {
        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;
        this.toggleLockOn = toggleLockOn;

        drive = Drive.getInstance();
        lockOnPID = new PIDController(DriveConstants.lockKP, 0, 0);
        lockOnPID.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetModulesToAbsolute();
    }

    @Override
    public void execute() {
        boolean toggle = toggleLockOn.getAsBoolean();
        if (toggle && !prevToggle) {lockOn = !lockOn;}
        prevToggle = toggle;

        // Any manual rotation cancels lock-on.
        if (rotation.getAsDouble() != 0) {lockOn = false;}

        Optional<Rotation2d> target = lockOn ? drive.getHeadingToHub() : Optional.empty();
        double rotSpeed = rotation.getAsDouble();
        if (target.isPresent()) {
            rotSpeed = MathUtil.clamp(
                    lockOnPID.calculate(drive.getHeading().getRadians(), target.get().getRadians()),
                    -DriveConstants.lockOnMaxSpeed, DriveConstants.lockOnMaxSpeed);
            if (Math.abs(rotSpeed) < DriveConstants.lockDeadband) {rotSpeed = 0;}
        }

        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        double sign = isRed ? -1.0 : 1.0;
        drive.drive(
                new Translation2d(sign * translationX.getAsDouble() * DriveConstants.maxSpeed,
                        sign * translationY.getAsDouble() * DriveConstants.maxSpeed),
                rotSpeed * DriveConstants.maxAngularVelocity,
                true,
                false);

        Logger.recordOutput("Drive/LockOn/Active", target.isPresent());
        Logger.recordOutput("Drive/LockOn/TargetAngleRad", target.map(Rotation2d::getRadians).orElse(0.0));
        Logger.recordOutput("Drive/LockOn/ErrorRad",
                target.map(t -> t.minus(drive.getHeading()).getRadians()).orElse(0.0));
    }
}

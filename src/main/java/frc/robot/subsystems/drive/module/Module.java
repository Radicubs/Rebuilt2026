package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.SwerveModuleConstants;
import org.littletonrobotics.junction.Logger;

public class Module {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public static Module create(int index, SwerveModuleConstants constants, boolean sim) {
        return new Module(sim ? new ModuleIOSim() : new ModuleIOReal(constants), index);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState.optimize(inputs.anglePosition);
        io.setAnglePosition(desiredState.angle);
        if (isOpenLoop) {
            io.setDriveDutyCycle(desiredState.speedMetersPerSecond / DriveConstants.maxSpeed);
        } else {
            io.setDriveVelocity(desiredState.speedMetersPerSecond);
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.driveVelocityMps, inputs.anglePosition);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePositionMeters, inputs.anglePosition);
    }

    public void resetToAbsolute() {
        io.resetToAbsolute();
    }

    public void stop() {
        io.stop();
    }
}

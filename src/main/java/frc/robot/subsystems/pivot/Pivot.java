package frc.robot.subsystems.pivot;

import frc.robot.constants.PivotConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

    private static Pivot INSTANCE;
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private final ProfiledPIDController pid;
    private final ArmFeedforward feedforward;
    private boolean moveToTargetAngle = false;

    public static Pivot getInstance(){
        if(INSTANCE == null) {INSTANCE = new Pivot();}
        return INSTANCE;
    }

    private Pivot(){

        io = RobotBase.isSimulation() ? new PivotIOSim() : new PivotIOReal();
        io.setPosition(PivotConstants.upPos);

        pid = new ProfiledPIDController(PivotConstants.PIDFeedforwardConstants.P, PivotConstants.PIDFeedforwardConstants.I, PivotConstants.PIDFeedforwardConstants.D, new TrapezoidProfile.Constraints(.5, 3));
        pid.setTolerance(PivotConstants.PIDFeedforwardConstants.pidTolerance);

        feedforward = new ArmFeedforward(PivotConstants.PIDFeedforwardConstants.S, PivotConstants.PIDFeedforwardConstants.G, PivotConstants.PIDFeedforwardConstants.V);

        PivotLogger.publish(this);

    }
    public double getPosition() {
        return inputs.positionRot;
    }
    public double getSpeed(){
        return inputs.appliedDuty;
    }
    public void setSpeed(double speed){
        moveToTargetAngle = false;
        io.setDutyCycle(speed);
    }
    public void setGoal(double targetRotation){
        cancelPID();
        moveToTargetAngle = true;
        pid.setGoal(new TrapezoidProfile.State(targetRotation, PivotConstants.pivotFinalVelocity));
    }
    public double getDesiredAngle() {
        return pid.getSetpoint().position;
    }
    double getSetpointVelocity() { return pid.getSetpoint().velocity; }
    boolean atGoal() { return pid.atGoal(); }
    boolean isMovingToTarget() { return moveToTargetAngle; }
    public void resetAngle(){
        io.setPosition(PivotConstants.downPos);
    }
    public void cancelPID(){
        io.setDutyCycle(0);
        moveToTargetAngle = false;
        pid.reset(inputs.positionRot);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        PivotLogger.log(this);

        if (moveToTargetAngle) {
            double motorSpeed = pid.calculate(getPosition());
            double feedforwardVal = feedforward.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);

            io.setDutyCycle(motorSpeed + feedforwardVal);

            if(pid.atGoal()){cancelPID();}
        }
    }
}

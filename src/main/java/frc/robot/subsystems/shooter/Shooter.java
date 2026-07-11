package frc.robot.subsystems.shooter;

import frc.robot.constants.ShooterConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private static Shooter INSTANCE;

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double mainSetpoint;
    private double topSetpoint;
    private double indexerSetpoint;
//
//    private double customMainShooterSpeed = ShooterConstants.CloseShootSpeeds.mainShooterRPS;
//    private double customTopShooterSpeed = ShooterConstants.CloseShootSpeeds.topShaftRPS;

    private double distanceToHub;
    private double regressionMainSpeeds;
    private double regressionTopSpeeds;

    public static Shooter getInstance() {
        if (INSTANCE == null) {INSTANCE = new Shooter();}
        return INSTANCE;
    }

    private Shooter() {
        io = RobotBase.isSimulation() ? new ShooterIOSim() : new ShooterIOReal();
        ShooterLogger.publish(this);
    }

    public double getRightShooterSpeed() {
        return inputs.rightVelocityRps;
    }

    public double getLeftShooterSpeed() {
        return inputs.leftVelocityRps;
    }

    public double getIndexerSpeed() {
        return inputs.indexerVelocityRps;
    }

    public double getTopShooterSpeed() {
        return inputs.topVelocityRps;
    }

    public void setShooterSpeeds(double mainShooterRPS, double topShaftRPS, double indexerRPS) {
        mainSetpoint = mainShooterRPS;
        topSetpoint = topShaftRPS;
        indexerSetpoint = indexerRPS;
        io.setShooterSpeeds(mainShooterRPS, topShaftRPS, indexerRPS);
    }

    public void setIndexerSpeed(double indexerRPS) {
        indexerSetpoint = indexerRPS;
        io.setIndexerSpeed(indexerRPS);
    }

//    public double getCustomMainShooterSpeed() {
//        return customMainShooterSpeed;
//    }
//
//    public double getCustomTopShooterSpeed() {
//        return customTopShooterSpeed;
//    }
//
//    public void adjustCustomSpeeds(double mainShooterChange, double topShooterChange) {
//        customMainShooterSpeed += mainShooterChange;
//        customTopShooterSpeed += topShooterChange;
//    }

    public double getRegressionMainSpeed() {
        return regressionMainSpeeds;
    }

    public double getRegressionTopSpeed() {
        return regressionTopSpeeds;
    }

    double getMainSetpoint() { return mainSetpoint; }
    double getTopSetpoint() { return topSetpoint; }
    double getIndexerSetpoint() { return indexerSetpoint; }
    double getDistanceToHub() { return distanceToHub; }

    public void cancelPID() {
        mainSetpoint = 0;
        topSetpoint = 0;
        indexerSetpoint = 0;
        io.stop();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (DriverStation.getAlliance().isPresent()) {
            // Distance to OUR alliance's hub (pose is field-absolute, so red must use the red hub).
            var hub = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                    ? FieldConstants.hubCenterRed
                    : FieldConstants.hubCenterBlue;
            distanceToHub = hub.getTranslation().getDistance(Drive.getInstance().getPose().getTranslation());
            regressionMainSpeeds = 5.48105 * distanceToHub + 27.26404;
            regressionTopSpeeds = -2.16514 * Math.pow(distanceToHub, 3) + 22.77548 * Math.pow(distanceToHub, 2) - 75.31997 * distanceToHub + 91.22797;

            if (regressionMainSpeeds > 60) {regressionMainSpeeds = 60;}
            if (regressionTopSpeeds > 25) {regressionTopSpeeds = 25;}
            if (distanceToHub > 5.2) {regressionTopSpeeds = 17.5;}
        }

        ShooterLogger.log(this);
    }
}

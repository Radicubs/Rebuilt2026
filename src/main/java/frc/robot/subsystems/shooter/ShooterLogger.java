package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

final class ShooterLogger {

    private ShooterLogger() {
    }

    static void publish(Shooter shooter) {
        SmartDashboard.putData("Right Shooter", b ->
                b.addDoubleProperty("Right Shooter Speed", shooter::getRightShooterSpeed, null));

        SmartDashboard.putData("Right Setpoint", b -> b.addDoubleProperty("Right setpoint", shooter::getMainSetpoint, null));

        SmartDashboard.putData("Left Shooter", b ->
                b.addDoubleProperty("Left Shooter Speed", shooter::getLeftShooterSpeed, null));

        SmartDashboard.putData("Indexer", b -> {
            b.addDoubleProperty("Indexer Speed", shooter::getIndexerSpeed, null);
        });

        SmartDashboard.putData("Top Shooter", b -> {
            b.addDoubleProperty("Top Shooter Speed", shooter::getTopShooterSpeed, null);
        });
//
//        SmartDashboard.putData("Custom Shots", b -> {
//            b.addDoubleProperty("Top Shooter Custom", shooter::getCustomTopShooterSpeed, null);
//            b.addDoubleProperty("Main Shooter Custom", shooter::getCustomMainShooterSpeed, null);
//        });
    }

        static void log(Shooter shooter){
            Logger.recordOutput("Shooter/Setpoint/Main", shooter.getMainSetpoint());
            Logger.recordOutput("Shooter/Setpoint/Top", shooter.getTopSetpoint());
            Logger.recordOutput("Shooter/Setpoint/Indexer", shooter.getIndexerSetpoint());
            Logger.recordOutput("Shooter/Error/Main", shooter.getMainSetpoint() - shooter.getLeftShooterSpeed());
            Logger.recordOutput("Shooter/Error/Top", shooter.getTopSetpoint() - shooter.getTopShooterSpeed());
            Logger.recordOutput("Shooter/Regression/Main", shooter.getRegressionMainSpeed());
            Logger.recordOutput("Shooter/Regression/Top", shooter.getRegressionTopSpeed());
            Logger.recordOutput("Shooter/DistanceToHub", shooter.getDistanceToHub());
        }
    }



package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

final class ShooterLogger {

    private ShooterLogger() {}

    static void publish(Shooter shooter) {
        SmartDashboard.putData("Right Shooter", b ->
                b.addDoubleProperty("Right Shooter Speed", shooter::getRightShooterSpeed, null));

        SmartDashboard.putData("Left Shooter", b ->
                b.addDoubleProperty("Left Shooter Speed", shooter::getLeftShooterSpeed, null));

        SmartDashboard.putData("Indexer", b -> {
            b.addDoubleProperty("Indexer Speed", shooter::getIndexerSpeed, null);
        });

        SmartDashboard.putData("Top Shooter", b -> {
            b.addDoubleProperty("Top Shooter Speed", shooter::getTopShooterSpeed, null);
        });

        SmartDashboard.putData("Custom Shots", b -> {
            b.addDoubleProperty("Top Shooter Custom", shooter::getCustomTopShooterSpeed, null);
            b.addDoubleProperty("Main Shooter Custom", shooter::getCustomMainShooterSpeed, null);
        });
    }
}

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.field.FieldConstants;
import frc.robot.subsystems.swerve.Swerve;

public class Shooter extends SubsystemBase {

    private static Shooter INSTANCE;

    private final TalonFXConfiguration indexerConfig;
    private final TalonFXConfiguration topShooterConfig;

    private final TalonFX indexer;
    private final TalonFX topShooter;

    private final TalonFXConfiguration leftConfig;
    private final TalonFXConfiguration rightConfig;

    private final TalonFX leftShooter;
    private final TalonFX rightShooter;

    private final VelocityVoltage leftShooterVel = new VelocityVoltage(0), rightShooterVel = new VelocityVoltage(0), indexerVel = new VelocityVoltage(0), topShooterVel = new VelocityVoltage(0);


    private final SimpleMotorFeedforward indexerFF = new SimpleMotorFeedforward(ShooterConstants.IndexerPIDFeedforwardConstants.kS, ShooterConstants.IndexerPIDFeedforwardConstants.kV, ShooterConstants.IndexerPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward topShooterFF = new SimpleMotorFeedforward(ShooterConstants.TopShooterPIDFeedforwardConstants.kS, ShooterConstants.TopShooterPIDFeedforwardConstants.kV, ShooterConstants.TopShooterPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward leftMainShooterFF = new SimpleMotorFeedforward(ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kS, ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kV, ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward rightMainShooterFF = new SimpleMotorFeedforward(ShooterConstants.MainRightShooterPIDFeedforwardConstants.kS, ShooterConstants.MainRightShooterPIDFeedforwardConstants.kV, ShooterConstants.MainRightShooterPIDFeedforwardConstants.kA);

    private double customMainShooterSpeed;
    private double customTopShooterSpeed;

    private double distanceToHub;

    private double regressionMainSpeeds;
    private double regressionTopSpeeds;

    public static Shooter getInstance() {
        if (INSTANCE == null) {INSTANCE = new Shooter();}
        return INSTANCE;
    }

    private Shooter() {

        customMainShooterSpeed = ShooterConstants.CloseShootSpeeds.mainShooterRPS;
        customTopShooterSpeed = ShooterConstants.CloseShootSpeeds.topShaftRPS;

        // Indexer Config
        indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.indexerEnableCurrentLimit;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.indexerShooterCurrentLimit;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed

        indexerConfig.Slot0.kP = ShooterConstants.IndexerPIDFeedforwardConstants.kP;
        indexerConfig.Slot0.kI = ShooterConstants.IndexerPIDFeedforwardConstants.kI;
        indexerConfig.Slot0.kD = ShooterConstants.IndexerPIDFeedforwardConstants.kD;


        // Top Shooter Config
        topShooterConfig = new TalonFXConfiguration();
        topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topShooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.topShooterEnableCurrentLimit;
        topShooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.topShooterCurrentLimit;
        topShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed

        topShooterConfig.Slot0.kP = ShooterConstants.TopShooterPIDFeedforwardConstants.kP;
        topShooterConfig.Slot0.kI = ShooterConstants.TopShooterPIDFeedforwardConstants.kI;
        topShooterConfig.Slot0.kD = ShooterConstants.TopShooterPIDFeedforwardConstants.kD;


        // Left Motor Config
        leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.shooterEnableCurrentLimit;
        leftConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.shooterCurrentLimit;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed

        leftConfig.Slot0.kP = ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kP;
        leftConfig.Slot0.kI = ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kI;
        leftConfig.Slot0.kD = ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kD;

        // Right Motor Config
        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.shooterEnableCurrentLimit;
        rightConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.shooterCurrentLimit;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO: Change if needed

        rightConfig.Slot0.kP = ShooterConstants.MainRightShooterPIDFeedforwardConstants.kP;
        rightConfig.Slot0.kI = ShooterConstants.MainRightShooterPIDFeedforwardConstants.kI;
        rightConfig.Slot0.kD = ShooterConstants.MainRightShooterPIDFeedforwardConstants.kD;

        // Indexer
        indexer = new TalonFX(ShooterConstants.indexerCID);
        indexer.getConfigurator().apply(indexerConfig);

        // Top Shooter
        topShooter = new TalonFX(ShooterConstants.topShooterCID);
        topShooter.getConfigurator().apply(topShooterConfig);

        // Left Shooter
        leftShooter = new TalonFX(ShooterConstants.leftShooterCID);
        leftShooter.getConfigurator().apply(leftConfig);

        // Right Shooter
        rightShooter = new TalonFX(ShooterConstants.rightShooterCID);
        rightShooter.getConfigurator().apply(rightConfig);

        ShooterLogger.publish(this);


    }

    public double getRightShooterSpeed() {
        return rightShooter.getVelocity().getValue().in(Units.RotationsPerSecond);
    }

    public double getLeftShooterSpeed() {
        return leftShooter.getVelocity().getValue().in(Units.RotationsPerSecond);
    }

    public double getIndexerSpeed() {
        return indexer.getVelocity().getValue().in(Units.RotationsPerSecond);
    }

    public double getTopShooterSpeed() {
        return topShooter.getVelocity().getValue().in(Units.RotationsPerSecond);
    }

    public void setShooterSpeeds(double mainShooterRPS, double topShaftRPS, double indexerRPS) {

        indexerVel.Velocity = indexerRPS;
        indexerVel.FeedForward = indexerFF.calculate(indexerRPS);

        topShooterVel.Velocity = topShaftRPS;
        topShooterVel.FeedForward = topShooterFF.calculate(topShaftRPS);

        leftShooterVel.Velocity = mainShooterRPS;
        leftShooterVel.FeedForward = leftMainShooterFF.calculate(mainShooterRPS);

        rightShooterVel.Velocity = mainShooterRPS;
        rightShooterVel.FeedForward = rightMainShooterFF.calculate(mainShooterRPS);

        rightShooter.setControl(rightShooterVel);

        leftShooter.setControl(leftShooterVel);

        indexer.setControl(indexerVel);

        topShooter.setControl(topShooterVel);
    }

    public void setIndexerSpeed(double indexerRPS) {

        indexerVel.Velocity = indexerRPS;
        indexerVel.FeedForward = indexerFF.calculate(indexerRPS);

        indexer.setControl(indexerVel);
    }

    public double getCustomMainShooterSpeed() {
        return customMainShooterSpeed;
    }
    public double getCustomTopShooterSpeed() {
        return customTopShooterSpeed;
    }
    public void adjustCustomSpeeds(double mainShooterChange, double topShooterChange) {
        customMainShooterSpeed += mainShooterChange;
        customTopShooterSpeed += topShooterChange;
    }

    public double getRegressionMainSpeed() {
        return regressionMainSpeeds;
    }
    public double getRegressionTopSpeed() {
        return regressionTopSpeeds;
    }
    public void cancelPID() {
        leftShooter.set(0);
        rightShooter.set(0);

        indexer.set(0);

        topShooter.set(0);
    }

    @Override
    public void periodic() {

        if (DriverStation.getAlliance().isPresent()) {
            // Distance to OUR alliance's hub (pose is field-absolute, so red must use the red hub).
            var hub = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                    ? FieldConstants.hubCenterRed
                    : FieldConstants.hubCenterBlue;
            distanceToHub = hub.getTranslation().getDistance(Swerve.getInstance().getPose().getTranslation());
            regressionMainSpeeds = 5.48105 * distanceToHub + 27.26404;
            regressionTopSpeeds = -2.16514*Math.pow(distanceToHub, 3) + 22.77548*Math.pow(distanceToHub, 2) - 75.31997*distanceToHub + 91.22797;

            if(regressionMainSpeeds > 60){regressionMainSpeeds = 60;}
            if(regressionTopSpeeds > 25){regressionTopSpeeds = 25;}
            if(distanceToHub > 5.2) {regressionTopSpeeds = 17.5;}
        }
    }
}

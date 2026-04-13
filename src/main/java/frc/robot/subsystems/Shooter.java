package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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


    private final SimpleMotorFeedforward indexerFF = new SimpleMotorFeedforward(Constants.Shooter.IndexerPIDFeedforwardConstants.kS, Constants.Shooter.IndexerPIDFeedforwardConstants.kV, Constants.Shooter.IndexerPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward topShooterFF = new SimpleMotorFeedforward(Constants.Shooter.TopShooterPIDFeedforwardConstants.kS, Constants.Shooter.TopShooterPIDFeedforwardConstants.kV, Constants.Shooter.TopShooterPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward leftMainShooterFF = new SimpleMotorFeedforward(Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kS, Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kV, Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward rightMainShooterFF = new SimpleMotorFeedforward(Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kS, Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kV, Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kA);

    private double customShootSpeed;
    private double customTopshaftSpeed;

    private Transform2d toTarget;
    private double distanceToHub;
    private double regressionMainSpeeds;
    private double regressionTopSpeeds;

    public static Shooter getInstance() {
        if (INSTANCE == null) {INSTANCE = new Shooter();}
        return INSTANCE;
    }

    private Shooter() {

        customShootSpeed = Constants.Shooter.CloseShootSpeeds.mainShooterRPS;
        customTopshaftSpeed = Constants.Shooter.CloseShootSpeeds.topShaftRPS;

        // Indexer Config
        indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.indexerEnableCurrentLimit;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.indexerShooterCurrenLimit;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed

        indexerConfig.Slot0.kP = Constants.Shooter.IndexerPIDFeedforwardConstants.kP;
        indexerConfig.Slot0.kI = Constants.Shooter.IndexerPIDFeedforwardConstants.kI;
        indexerConfig.Slot0.kD = Constants.Shooter.IndexerPIDFeedforwardConstants.kD;


        // Top Shooter Config
        topShooterConfig = new TalonFXConfiguration();
        topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topShooterConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.topShooterEnableCurrentLimit;
        topShooterConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.topShooterCurrentLimit;
        topShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed

        topShooterConfig.Slot0.kP = Constants.Shooter.TopShooterPIDFeedforwardConstants.kP;
        topShooterConfig.Slot0.kI = Constants.Shooter.TopShooterPIDFeedforwardConstants.kI;
        topShooterConfig.Slot0.kD = Constants.Shooter.TopShooterPIDFeedforwardConstants.kD;


        // Left Motor Config
        leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.shooterEnableCurrentLimit;
        leftConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.shooterCurrentLimit;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed

        leftConfig.Slot0.kP = Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kP;
        leftConfig.Slot0.kI = Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kI;
        leftConfig.Slot0.kD = Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kD;

        // Right Motor Config
        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.shooterEnableCurrentLimit;
        rightConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.shooterCurrentLimit;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO: Change if needed

        rightConfig.Slot0.kP = Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kP;
        rightConfig.Slot0.kI = Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kI;
        rightConfig.Slot0.kD = Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kD;

        // Indexer
        indexer = new TalonFX(Constants.Shooter.indexerCID);
        indexer.getConfigurator().apply(indexerConfig);

        /*
        SmartDashboard.putData("Indexer", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Indexer Speed", () -> getIndexerSpeed(), null);
            }
        });
         */

        // Top Shooter
        topShooter = new TalonFX(Constants.Shooter.topShooterCID);
        topShooter.getConfigurator().apply(topShooterConfig);

        /*
        SmartDashboard.putData("Top Shooter", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Top Shooter Speed", () -> getTopShooterSpeed(), null);
            }
        });
         */

        // Left Shooter
        leftShooter = new TalonFX(Constants.Shooter.leftShooterCID);
        leftShooter.getConfigurator().apply(leftConfig);

        // Right Shooter
        rightShooter = new TalonFX(Constants.Shooter.rightShooterCID);
        rightShooter.getConfigurator().apply(rightConfig);


        SmartDashboard.putData("Right Shooter", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Right Shooter Speed", () -> getRightShooterSpeed(), null);
            }
        });

        SmartDashboard.putData("Left Shooter", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Left Shooter Speed", () -> getLeftShooterSpeed(), null);
            }
        });

        SmartDashboard.putData("Indexer", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Indexer Speed", () -> getIndexerSpeed(), null);
                builder.addDoubleProperty("Indexer Desired Speed", () -> indexerVel.Velocity, null);
            }
        });

        SmartDashboard.putData("Top Shooter", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Top Shooter Speed", () -> getTopShooterSpeed(), null);
                builder.addDoubleProperty("Top Shooter Desired Speed", () -> topShooterVel.Velocity, null);
            }
        });


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

    public void cancelPID() {
        leftShooter.set(0);
        rightShooter.set(0);

        indexer.set(0);

        topShooter.set(0);
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

    public void setMainSpeeds(double mainShooterRPS, double topShaftRPS) {

        topShooterVel.Velocity = topShaftRPS;
        topShooterVel.FeedForward = topShooterFF.calculate(topShaftRPS);

        leftShooterVel.Velocity = mainShooterRPS;
        leftShooterVel.FeedForward = leftMainShooterFF.calculate(mainShooterRPS);

        rightShooterVel.Velocity = mainShooterRPS;
        rightShooterVel.FeedForward = rightMainShooterFF.calculate(mainShooterRPS);

        rightShooter.setControl(rightShooterVel);
        leftShooter.setControl(leftShooterVel);
        topShooter.setControl(topShooterVel);
    }

    public void setIndexerSpeed(double indexerRPS) {

        indexerVel.Velocity = indexerRPS;
        indexerVel.FeedForward = indexerFF.calculate(indexerRPS);

        indexer.setControl(indexerVel);
    }

    public void closeRamp() {
        setShooterSpeeds(Constants.Shooter.CloseShootSpeeds.mainShooterRPS, Constants.Shooter.CloseShootSpeeds.topShaftRPS, 0);
    }

    public void trenchRamp() {
        setShooterSpeeds(Constants.Shooter.TrenchShootSpeeds.mainShooterRPS, Constants.Shooter.TrenchShootSpeeds.topShaftRPS, 0);
    }

    public void passRamp() {
        setShooterSpeeds(Constants.Shooter.PassSpeeds.mainShooterRPS, Constants.Shooter.PassSpeeds.topShaftRPS, 0);
    }

    public void customRamp() {
        setShooterSpeeds(customShootSpeed, customTopshaftSpeed, 0);
    }

    public void changeShooterSpeeds(double changeAmount) {
        customShootSpeed += changeAmount;
    }

    public void changeIndexerSpeeds(double changeAmount) {
        customTopshaftSpeed += changeAmount;
    }

    public void stop() {
        setShooterSpeeds(0, 0, 0);
    }

    public void regressionRamp() {
        setMainSpeeds(regressionMainSpeeds, regressionTopSpeeds);
    }

    @Override
    public void periodic() {

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                toTarget = Swerve.getInstance().getPose().minus(Constants.Field.hubCenterBlue);
            } else {
                toTarget = Swerve.getInstance().getPose().minus(Constants.Field.hubCenterRed);
            }

            distanceToHub = (Math.sqrt(toTarget.getX() * toTarget.getX() + toTarget.getY() * toTarget.getY())) - .75;
            regressionMainSpeeds = 65.185 / (1 + Math.exp(-0.358845 * distanceToHub + -0.291163));
            regressionTopSpeeds = 10.36912 / (1 + Math.exp(-(1.83055 * distanceToHub - 2.48482)));
        }
    }
}

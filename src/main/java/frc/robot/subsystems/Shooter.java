package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private static Shooter INSTANCE;

    private SparkMax indexer;

    private final PIDController indexerController;
    private final SimpleMotorFeedforward indexerFeedforward;

    private SparkMax topShooter;
    private final PIDController topShooterController;
    private final SimpleMotorFeedforward topShooterFeedforward;

    private TalonFXConfiguration leftConfig;
    private TalonFXConfiguration rightConfig;

    private TalonFX leftShooter;
    private TalonFX rightShooter;

    private final VelocityVoltage leftShooterVel = new VelocityVoltage(0), rightShooterVel = new VelocityVoltage(0);
    private final SimpleMotorFeedforward leftMainShooterFF = new SimpleMotorFeedforward(Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kS, Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kV, Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kA);

    private final SimpleMotorFeedforward rightMainShooterFF = new SimpleMotorFeedforward(Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kS, Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kV, Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kA);

    private boolean enablePID = false;

    public static Shooter getInstance() {
        if(INSTANCE == null) {INSTANCE = new Shooter();}
        return INSTANCE;
    }

    private Shooter(){
        // Indexer
        indexer = new SparkMax(Constants.Shooter.indexerCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig indexerConfigs = new SparkMaxConfig();
        indexerConfigs.inverted(true);
        indexerConfigs.idleMode(SparkBaseConfig.IdleMode.kCoast);
        indexerConfigs.secondaryCurrentLimit(35);
        indexer.configure(indexerConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        indexerController = new PIDController(Constants.Shooter.IndexerPIDFeedforwardConstants.kP, Constants.Shooter.IndexerPIDFeedforwardConstants.kI, Constants.Shooter.IndexerPIDFeedforwardConstants.kD);
        indexerFeedforward = new SimpleMotorFeedforward(Constants.Shooter.IndexerPIDFeedforwardConstants.kS, Constants.Shooter.IndexerPIDFeedforwardConstants.kV, Constants.Shooter.IndexerPIDFeedforwardConstants.kA);

        // Top Shooter Axle
        topShooter = new SparkMax(Constants.Shooter.topShooterCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig topShooterConfig = new SparkMaxConfig();
        topShooterConfig.inverted(true); //TODO: Change if needed
        topShooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        topShooterConfig.secondaryCurrentLimit(35);
        topShooter.configure(topShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        topShooterController = new PIDController(Constants.Shooter.TopShooterPIDFeedforwardConstants.kP, Constants.Shooter.TopShooterPIDFeedforwardConstants.kI, Constants.Shooter.TopShooterPIDFeedforwardConstants.kD);
        topShooterFeedforward = new SimpleMotorFeedforward(Constants.Shooter.TopShooterPIDFeedforwardConstants.kS, Constants.Shooter.TopShooterPIDFeedforwardConstants.kV, Constants.Shooter.TopShooterPIDFeedforwardConstants.kA);


        // Left Motor Config
        leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.shooterEnableCurrentLimit;
        leftConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.shooterCurrentLimit;
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO: Change if needed

        leftConfig.Slot0.kP = Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kP;
        leftConfig.Slot0.kI = Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kI;
        leftConfig.Slot0.kD = Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kD;

        // Right Motor Config
        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.shooterEnableCurrentLimit;
        rightConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.shooterCurrentLimit;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO: Change if needed

        rightConfig.Slot0.kP = Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kP;
        rightConfig.Slot0.kI = Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kI;
        rightConfig.Slot0.kD = Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kD;


        // Left Shooter
        leftShooter = new TalonFX(Constants.Shooter.leftShooterCID);
        leftShooter.getConfigurator().apply(leftConfig);

        // Right Shooter
        rightShooter = new TalonFX(Constants.Shooter.rightShooterCID);
        rightShooter.getConfigurator().apply(rightConfig);
    }

    public double getIndexerSpeed(){
        return indexer.getEncoder().getVelocity()/60.0;
    }
    public double getTopShooterSpeed(){
        return topShooter.getEncoder().getVelocity()/60.0;
    }

    public void setIndexerSetpoint(double indexerTargetRPS){
        indexerController.setSetpoint(indexerTargetRPS);
    }

    public void setTopShooterSetpoint(double topShooterTargetRPS){
        indexerController.setSetpoint(topShooterTargetRPS);
    }

    public void cancelPID(){
        leftShooter.set(0);
        rightShooter.set(0);

        indexer.set(0);
        indexerController.reset();

        topShooter.set(0);
        topShooterController.reset();

        enablePID = false;
    }

    public void setShooterSpeeds(double mainShooterRPS, double topShaftRPS, double indexerRPS){
        indexerController.setSetpoint(indexerRPS);
        //topShooterController.setSetpoint(topShaftRPS);

        leftShooterVel.Velocity = mainShooterRPS;
        leftShooterVel.FeedForward = leftMainShooterFF.calculate(mainShooterRPS);

        rightShooterVel.Velocity = mainShooterRPS;
        rightShooterVel.FeedForward = rightMainShooterFF.calculate(mainShooterRPS);

        //rightShooter.setControl(rightShooterVel);
        leftShooter.setControl(leftShooterVel);

        enablePID = true;
    }


    @Override
    public void periodic() {

        if(enablePID){
            double indexerMotorSpeed = indexerController.calculate(getIndexerSpeed());
            double indexerFeedForwardVal = indexerFeedforward.calculate(indexerController.getSetpoint());
            indexer.set(indexerMotorSpeed + indexerFeedForwardVal);

            double topShooterMotorSpeed = topShooterController.calculate(getTopShooterSpeed());
            double topShooterFeedForwardVal = topShooterFeedforward.calculate(topShooterController.getSetpoint());
            topShooter.set(topShooterMotorSpeed + topShooterFeedForwardVal);
        }

        SmartDashboard.putNumber("Left Shooter Speed", leftShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Shooter Speed", rightShooter.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Indexer speed", indexer.getEncoder().getVelocity()/60.0);

        SmartDashboard.putNumber("Top Shooter", topShooter.getEncoder().getVelocity()/60.0);

        SmartDashboard.putNumber("Left shooter current draw", leftShooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right shooter current draw", rightShooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Top shooter current draw", topShooter.getOutputCurrent());
        SmartDashboard.putNumber("Indexer current draw", indexer.getOutputCurrent());


    }
}

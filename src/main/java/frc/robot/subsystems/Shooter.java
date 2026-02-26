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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private static Shooter INSTANCE;

    private SparkMax indexer;
    private SparkMax topShooter;

    private TalonFXConfiguration leftConfig;
    private TalonFXConfiguration rightConfig;

    private TalonFX leftShooter;
    private TalonFX rightShooter;

    private final VelocityVoltage leftShooterVel = new VelocityVoltage(0), rightShooterVel = new VelocityVoltage(0);
    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(Constants.Shooter.kS, Constants.Shooter.kV, Constants.Shooter.kA);

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
        indexer.configure(indexerConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Top Shooter Axle
        topShooter = new SparkMax(Constants.Shooter.topShooterCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig topShooterConfig = new SparkMaxConfig();
        topShooterConfig.inverted(true); //TODO: Change if needed
        topShooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        topShooter.configure(topShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Motor Config
        leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.shooterEnableCurrentLimit;
        leftConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.shooterCurrentLimit;

        leftConfig.Slot0.kP = Constants.Shooter.kP;
        leftConfig.Slot0.kI = Constants.Shooter.kI;
        leftConfig.Slot0.kD = Constants.Shooter.kD;

        rightConfig = leftConfig.clone();
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO: Change if needed

        // Left Shooter
        leftShooter = new TalonFX(Constants.Shooter.leftShooterCID);
        leftShooter.getConfigurator().apply(leftConfig);

        // Right Shooter
        rightShooter = new TalonFX(Constants.Shooter.rightShooterCID);
        rightShooter.getConfigurator().apply(rightConfig);
    }

    public void setShooterSpeeds(double mainShooterRPM, double topShaftSpeed, double indexerSpeed){
        indexer.set(indexerSpeed);
        topShooter.set(topShaftSpeed);

        leftShooterVel.Velocity = mainShooterRPM;
        leftShooterVel.FeedForward = shooterFF.calculate(mainShooterRPM);

        rightShooterVel.Velocity = mainShooterRPM;
        rightShooterVel.FeedForward = shooterFF.calculate(mainShooterRPM);

        rightShooter.setControl(rightShooterVel);
        leftShooter.setControl(leftShooterVel);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Shooter Speed", leftShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Shooter Speed", rightShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Top Shooter Speed", topShooter.getEncoder().getVelocity());
    }
}

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private static Intake INSTANCE;

    private SparkMax intakeMotor;

    public static Intake getInstance(){
        if(INSTANCE == null) {INSTANCE = new Intake();}
        return INSTANCE;
    }

    private Intake(){
        intakeMotor = new SparkMax(Constants.Intake.intakeMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.inverted(true);
        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }


}

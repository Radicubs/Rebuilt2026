package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transfer extends SubsystemBase {

    private static Transfer INSTANCE;

    private SparkMax beltMotor;

    public static Transfer getInstance(){
        if(INSTANCE == null) {INSTANCE = new Transfer();}
        return INSTANCE;
    }

    private Transfer(){
        beltMotor = new SparkMax(Constants.Transfer.beltMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
        beltMotorConfig.inverted(false); //TODO: CHANGE IF NEEDED
        beltMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTransferSpeeds(double speed){
        beltMotor.set(speed);
    }

}

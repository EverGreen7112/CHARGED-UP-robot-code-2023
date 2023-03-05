package frc.robot.commands.unused;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MotionMagicArmPID extends CommandBase {
    private TalonFX m_first;
    // private TalonFX /m_seccond;


    public MotionMagicArmPID(){
        m_first = Arm.getInstance().getFirst();
        //m_seccond = Arm.getInstance().getSecond();

        m_first.configFactoryDefault();
        //m_seccond.configFactoryDefault();
        double p =4,i=0.000006,d=0,f=2;;
        m_first.config_kP(0, p);
        m_first.config_kI(0, i);
        m_first.config_kD(0,d);
        m_first.config_kF(0, f);
        //m_seccond.config_kP(0, p);
        //m_seccond.config_kI(0, i);
        //m_seccond.config_kD(0, d);
        //m_seccond.config_kF(0, f);

        m_first.setNeutralMode(NeutralMode.Brake);
        //m_seccond.setNeutralMode(NeutralMode.Brake);

        m_first.configMotionAcceleration(10000);
        //m_seccond.configMotionAcceleration(10000);
        
        m_first.configMotionCruiseVelocity(20000);
        //m_seccond.configMotionCruiseVelocity(20000);
        
        m_first.configForwardSoftLimitThreshold(89000, 30);
        m_first.configForwardSoftLimitEnable(true, 6000);

        //m_seccond.configForwardSoftLimitThreshold(89000, 30);
        //m_seccond.configForwardSoftLimitEnable(true, 6000);

        m_first.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 30);
        m_first.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 30);

        //m_seccond.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 30);
        //m_seccond.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 30);

        m_first.configNeutralDeadband(0.1);
        //m_seccond.configNeutralDeadband(0.1);
 
    }
    public double getFirstPosition(){
        return m_first.getSelectedSensorPosition(); 
    }
    // public double getSecondPosition(){
    //     return //m_seccond.getSelectedSensorPosition(); 

    // }
    public void setSecondPosition(int position){
        //m_seccond.set(TalonFXControlMode.MotionMagic, position);
    }
    public void setFirstPosition(){
        //m_seccond.set(TalonFXControlMode.MotionMagic, getSecondPosition());
    }

    public void execute(){
        setSecondPosition(-1);
    }
}

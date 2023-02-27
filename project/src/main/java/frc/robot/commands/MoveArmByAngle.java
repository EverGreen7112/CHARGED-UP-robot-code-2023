package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmValues;
import frc.robot.Constants.PidValues;
import frc.robot.subsystems.Arm;

public class MoveArmByAngle extends CommandBase{

    TalonFX m_first;
    TalonFX m_second;
    double m_firstArmAngle, m_secondArmAngle;
    double m_firstArmTarget, m_secondArmTarget;

    public MoveArmByAngle(double firstTargetAngle, double secondTargetAngle) {
        addRequirements(Arm.getInstance());
        m_first = Arm.getInstance().getFirst();
        m_second = Arm.getInstance().getSecond();
        m_firstArmTarget = firstTargetAngle;
        m_secondArmTarget = secondTargetAngle;
    }
    @Override
    public void initialize() {
        stage=0;
    }
    private int stage=0;
    @Override
    public void execute() {

        m_firstArmAngle = Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
        m_secondArmAngle = Constants.Conversions.ticksToAngle(m_second.getSelectedSensorPosition(), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
        SmartDashboard.putNumber("stage", 0);
        m_first.set(TalonFXControlMode.Position, Constants.Conversions.angleToTicks(m_firstArmTarget, Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION));
        if(stage == 0){
            if(m_firstArmAngle < Constants.ArmValues.LIMIT_TOLERANCE +  m_firstArmTarget && m_firstArmAngle > m_firstArmTarget - Constants.ArmValues.LIMIT_TOLERANCE){
                m_second.config_kF(0, PidValues.SECOND_ARM_KF);
                stage =1;
            }
            m_second.set(TalonFXControlMode.Position, 0);
        }
        if(stage ==1){
            m_second.set(TalonFXControlMode.Position, Constants.Conversions.angleToTicks(m_secondArmTarget, Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION));
            
            if(m_secondArmAngle<=-75 && m_secondArmAngle >= -90){
                stage=2;
            }
        }
        if(stage ==2){
            m_second.set(TalonFXControlMode.PercentOutput, -0.17);
            if(m_secondArmAngle < 4 +  m_secondArmTarget && m_secondArmAngle > m_secondArmTarget - 4){
                stage =3;  
            }
        }
        if(stage ==3){
            m_second.set(TalonFXControlMode.PercentOutput,m_firstArmTarget <0 ? PidValues.SECOND_ARM_STALL_SPEED: -1*PidValues.SECOND_ARM_STALL_SPEED);//check whether positive power or negative
        }
           
       
    }

    @Override
    public boolean isFinished() {
      return m_secondArmAngle < 4 +  m_secondArmTarget && m_secondArmAngle > m_secondArmTarget - 4;
    }
    @Override
    public void end(boolean interrupted) {
        m_second.config_kF(0, 0);
        m_second.set(TalonFXControlMode.PercentOutput, 0);
        

    }

}

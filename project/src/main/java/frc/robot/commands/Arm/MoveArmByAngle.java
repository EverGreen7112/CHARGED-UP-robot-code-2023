package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PidValues;
import frc.robot.subsystems.Arm;

public class MoveArmByAngle extends CommandBase{

    TalonFX m_first;
    TalonFX m_second;
    double m_firstArmAngle, m_secondArmAngle;
    double m_firstArmTarget, m_secondArmTarget;
    boolean m_finished;
    boolean doNothing =false;
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
        m_finished = false;
        double ang = Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
        if(ang >30 && m_firstArmTarget < 0){
            doNothing = true;
        }
        if(ang <-30 && m_firstArmTarget > 0){
            doNothing = true;
        }

    }
    private int stage=0;
    @Override
    public void execute() {
            if(!doNothing){ 
            SmartDashboard.putBoolean("finished", m_finished);
            m_firstArmAngle = Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
            m_secondArmAngle = Constants.Conversions.ticksToAngle(m_second.getSelectedSensorPosition(), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
            SmartDashboard.putNumber("stage", stage);
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
            }
        }
    }

    @Override
    public boolean isFinished() {
    //   return (m_secondArmAngle < 4 +  m_secondArmTarget && m_secondArmAngle > m_secondArmTarget - 4)||m_secondArmAngle>180;
          return (m_secondArmAngle < 7 +  m_secondArmTarget && m_secondArmAngle > m_secondArmTarget - 7)||doNothing;

    }
    @Override
    public void end(boolean interrupted) {
        if(!doNothing) { 
            m_second.config_kF(0, Constants.PidValues.SECOND_ARM_KF);
            //m_second.set(TalonFXControlMode.PercentOutput,m_firstArmTarget <0 ? Gripper.getInstance().getCurGamePiece(): -1*Gripper.getInstance().getCurGamePiece().getKstall());//check whether positive power or negative
            double stallTarget = (m_firstArmTarget < 0) ? Constants.PidValues.SECOND_ARM_STALL_SPEED :  -Constants.PidValues.SECOND_ARM_STALL_SPEED ;
            m_second.set(TalonFXControlMode.PercentOutput , stallTarget);
            m_finished = true;       
            SmartDashboard.putBoolean("finished", m_finished);
        }

    }

}

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    public void execute() {

        m_firstArmAngle = Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
        m_secondArmAngle = Constants.Conversions.ticksToAngle(m_second.getSelectedSensorPosition(), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
    
        m_first.set(TalonFXControlMode.Position, Constants.Conversions.angleToTicks(m_firstArmTarget, Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION));
        if(m_firstArmAngle < Constants.ArmValues.LIMIT_TOLERANCE +  m_firstArmTarget && m_firstArmAngle > m_firstArmTarget - Constants.ArmValues.LIMIT_TOLERANCE){
            m_second.set(TalonFXControlMode.Position, Constants.Conversions.angleToTicks(m_secondArmTarget, Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION));
        }
       
    }

    @Override
    public boolean isFinished() {
      return m_secondArmAngle < 4 +  m_secondArmTarget && m_secondArmAngle > m_secondArmTarget - 4;
    }

}

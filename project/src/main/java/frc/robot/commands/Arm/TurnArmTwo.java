package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class TurnArmTwo extends CommandBase{
    TalonFX m_second;
    double m_armAngle;
    double m_armTarget;

    public TurnArmTwo(double targetAngle) {
        addRequirements(Arm.getInstance());
        m_second = Arm.getInstance().getSecond();
        m_armTarget = targetAngle;
        
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
           
            m_armAngle = Constants.Conversions.ticksToAngle(m_second.getSelectedSensorPosition(), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
            m_second.set(TalonFXControlMode.Position, Constants.Conversions.angleToTicks(m_armTarget, Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION));
            
    
    }

    @Override
    public boolean isFinished() {
          return m_armAngle < 4 + m_armTarget && m_armAngle > m_armTarget - 4;
    }
 

    
}

package frc.robot.commands.unused;

import java.lang.constant.Constable;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmTwoStayInZero extends CommandBase{
    TalonFX m_second;
    public ArmTwoStayInZero(){
        m_second = Arm.getSecond();
        m_second.config_kP(0, Constants.PidValues.SECOND_ARM_KP * 2.5);
        
    }
    @Override
    public void execute() {
        m_second.set(TalonFXControlMode.Position, 0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        m_second.config_kP(0, Constants.PidValues.SECOND_ARM_KP);

    }
    
    
}

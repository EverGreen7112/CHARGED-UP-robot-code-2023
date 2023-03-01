package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmStayInPlace extends CommandBase{
    
    private TalonFX m_first;
    private TalonFX m_second;
    public ArmStayInPlace(){
        m_first = Arm.getInstance().getFirst();
        m_second = Arm.getInstance().getSecond();
    }

    @Override
    public void execute() {
        double m_firstArmAngle = Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
        double m_secondArmAngle = Constants.Conversions.ticksToAngle(m_second.getSelectedSensorPosition(), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
        m_first.set(TalonFXControlMode.Position, Constants.Conversions.angleToTicks(m_firstArmAngle, Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION));
        m_second.set(TalonFXControlMode.Position, Constants.Conversions.angleToTicks(m_secondArmAngle, Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION));
    }


}

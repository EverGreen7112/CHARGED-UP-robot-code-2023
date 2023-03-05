package frc.robot.commands.unused;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class JoyStickSum extends CommandBase{
    public double factor = 0.01;
    public double curval = 0;
    @Override
    public void execute() {
        if(!RobotContainer.m_operator.getRawButton(Constants.ButtonPorts.START) && !RobotContainer.m_operator.getRawButton(Constants.ButtonPorts.START)){
            if(curval < 1 && curval > -1){
                curval += RobotContainer.m_operator.getY() * factor;
            }
            Arm.getSecond().set(TalonFXControlMode.PercentOutput, curval);
        }
    }
    
}

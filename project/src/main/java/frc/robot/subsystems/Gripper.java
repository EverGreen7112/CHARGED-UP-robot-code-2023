package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase{
    
    private static Gripper m_instance;
    private TalonSRX m_motor;

    public Gripper(TalonSRX motor){
        m_motor = motor;
    }

    public static Gripper getInstance(){
        if (m_instance == null){
            m_instance = new Gripper(new TalonSRX(Constants.Ports.GRIPPER_PORT));
        }
        return m_instance;
    }

    public void moveGripper(double speed){
        m_motor.set(ControlMode.PercentOutput, speed);
    }

    public void closeGripper(){
        moveGripper(Constants.Speeds.GRIPPER_SPEED);
    }

    public void openGripper(){
        moveGripper(-Constants.Speeds.GRIPPER_SPEED);
    }

    public void stop(){
        moveGripper(0);
    }
}

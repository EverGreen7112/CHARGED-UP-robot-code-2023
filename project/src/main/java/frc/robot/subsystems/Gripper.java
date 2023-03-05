package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Gripper.CloseGripper;
import frc.robot.subsystems.Arm.ARMCONF;

public class Gripper extends SubsystemBase{
    
    private static Gripper m_instance;
    private TalonSRX m_motor;
    private DigitalInput m_open, m_close, m_cube;
    private boolean mode; //true is cone and false is else  

    public Gripper(TalonSRX motor){
        m_motor = motor;
        m_open = new DigitalInput(Constants.Ports.OPENED_LIMIT_SWITCH);
        m_close = new DigitalInput(Constants.Ports.CLOSED_LIMIT_SWITCH);
        m_cube = new DigitalInput(Constants.Ports.CUBE_LIMIT_SWITCH);
        mode =false;
        SmartDashboard.putBoolean("mode", mode);
    }

    public static Gripper getInstance(){
        if (m_instance == null){
            m_instance = new Gripper(new TalonSRX(Constants.Ports.GRIPPER_PORT));
        }
        return m_instance;
    }
    private Command lastCom = null;
    @Override
    public void periodic() {
        lastCom = Gripper.getInstance().getCurrentCommand() == null?lastCom :Gripper.getInstance().getCurrentCommand();
        SmartDashboard.putString("curCom", Gripper.getInstance().getCurrentCommand() == null?"null":Gripper.getInstance().getCurrentCommand().getClass().toString());
        if(lastCom!=null &&lastCom.getClass().toString().equals(CloseGripper.class.toString()) && Arm.getInstance().getConf() !=ARMCONF.MID){
            closeGripper();
        }else if(Gripper.getInstance().getCurrentCommand() == null) {
            stop();
        }
    }
    public void moveGripper(double speed){
        m_motor.set(ControlMode.PercentOutput, speed);
    }
    

    public void closeGripper(){
        moveGripper(Constants.Speeds.GRIPPER_CLOSE_SPEED);
    }
    public void closeGripperCube(){
        moveGripper(Constants.Speeds.GRIPPER_CLOSE_CUBE);
    }

    public void openGripper(){
        moveGripper(Constants.Speeds.GRIPPER_OPEN_SPEED);
    }

    public void stop(){
        moveGripper(0);
    }

    public DigitalInput getOpened(){
        return m_open;
    }

    public DigitalInput getClosed(){
        return m_close;
    }

    public DigitalInput getCube(){
        return m_cube;
    }
    public boolean getCurrentMode(){
        return mode;
    }
    public void switchModes(){
        mode = !mode;
        SmartDashboard.putBoolean("mode", mode);
        if(mode){ 
        Constants.PidValues.SECOND_ARM_KP *=1.3;
        Constants.PidValues.SECOND_ARM_KI *=1.3;
        Constants.PidValues.SECOND_ARM_KD *=1.3;
        Constants.PidValues.SECOND_ARM_KF *=1.4;
        Constants.PidValues.SECOND_ARM_STALL_SPEED *=1.7;

        }else{
            Constants.PidValues.SECOND_ARM_KP =Constants.PidValues._SECOND_ARM_KP;
            Constants.PidValues.SECOND_ARM_KI= Constants.PidValues._SECOND_ARM_KI;
            Constants.PidValues.SECOND_ARM_KD =Constants.PidValues._SECOND_ARM_KD;
            Constants.PidValues.SECOND_ARM_KF =Constants.PidValues._SECOND_ARM_KF;
            Constants.PidValues.SECOND_ARM_STALL_SPEED =Constants.PidValues._SECOND_ARM_STALL_SPEED;
        }
    }
}

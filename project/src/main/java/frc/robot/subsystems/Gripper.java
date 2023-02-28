package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PidValues;
import frc.robot.commands.CloseGripper;
import frc.robot.subsystems.Arm.ARMCONF;

public class Gripper extends SubsystemBase{
    
    private static Gripper m_instance;
    private TalonSRX m_motor;
    private DigitalInput m_open, m_close, m_cube;

    public static enum GamePiece{
        NONE(PidValues.SECOND_ARM_STALL_SPEED,PidValues.SECOND_ARM_KF, PidValues.SECOND_ARM_KP,PidValues.SECOND_ARM_KI,PidValues.SECOND_ARM_KD),
        CONE(PidValues.SECOND_ARM_STALL_SPEED,PidValues.SECOND_ARM_KF,PidValues.SECOND_ARM_KP,PidValues.SECOND_ARM_KI,PidValues.SECOND_ARM_KD),
        CUBE(PidValues.SECOND_ARM_STALL_SPEED,PidValues.SECOND_ARM_KF,PidValues.SECOND_ARM_KP,PidValues.SECOND_ARM_KI,PidValues.SECOND_ARM_KD);
        // private double kstall,kf1,kp1,ki1,kd1,kf2,kp2,ki2,kd2;
        private double kstall,kf1,kp1,ki1,kd1;
        private GamePiece(double kstall,double kf1,double kp1,double ki1, double kd1) {
            this.kstall = 
            this.kf1 = kf1;
            this.kp1 = kp1;
            this.ki1 = ki1;
            this.kd1 = kd1;

        }
        public double getKstall() {
            return kstall;
        }
    
        public double getKf1() {
            return kf1;
        }
    
        public double getKp1() {
            return kp1;
        }
    
        public double getKi1() {
            return ki1;
        }
    
        public double getKd1() {
            return kd1;
        }
        // public GamePiece(double kstall,double kf1,double kp1,double ki1, double kd1,double kf2,double kp2,double ki2, double kd2) {
        //     this.kstall = 
        //     this.kf1 = kf1;
        //     this.kp1 = kp1;
        //     this.ki1 = ki1;
        //     this.kd1 = kd1;
        //     this.kf2 = kf2;
        //     this.kp2 = kp2;
        //     this.ki2 = ki2;
        //     this.kd2 = kd2;
        // }
    }
    public Gripper(TalonSRX motor){
        m_motor = motor;
        m_open = new DigitalInput(Constants.Ports.OPENED_LIMIT_SWITCH);
        m_close = new DigitalInput(Constants.Ports.CLOSED_LIMIT_SWITCH);
        m_cube = new DigitalInput(Constants.Ports.CUBE_LIMIT_SWITCH);
    }
    private GamePiece m_curPiece = GamePiece.NONE;
    public void setCurPiece(GamePiece piece){
        m_curPiece = piece;
        Arm.getInstance().setSecondFPID(piece.kf1, piece.kp1, piece.ki1,piece.kd1);
    }
    public GamePiece getCurGamePiece(){
        return m_curPiece;
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
        lastCom = (Gripper.getInstance().getCurrentCommand() == null) ? lastCom : Gripper.getInstance().getCurrentCommand();
        SmartDashboard.putString("curCom", Gripper.getInstance().getCurrentCommand() == null?"null":Gripper.getInstance().getCurrentCommand().getClass().toString());
        if(lastCom!=null &&lastCom.getClass().toString().equals(CloseGripper.class.toString()) && Arm.getInstance().getConf() != ARMCONF.MID){
            defualtCloseGripper();
        }
        else if(Gripper.getInstance().getCurrentCommand() == null) {
            stop();
        }
    }
    public void moveGripper(double speed){
        m_motor.set(ControlMode.PercentOutput, speed);
    }
    

    public void closeGripper(){
        moveGripper(Constants.Speeds.GRIPPER_CLOSE_SPEED);
    }
    public void defualtCloseGripper(){
        moveGripper(Constants.Speeds.GRIPPER_DEFUALT_CLOSE);
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
}

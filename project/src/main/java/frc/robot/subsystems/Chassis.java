package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase{
    private MotorControllerGroup rightMotors;
    private MotorControllerGroup leftMotors;

    public MotorController m_leftFrontEngine, m_leftMiddleEngine, m_leftBackEngine;
    public MotorController m_rightFrontEngine, m_rightMiddleEngine, m_rightBackEngine;

    private static Chassis m_instance=null;

    public Chassis(){
        m_leftFrontEngine   = new CANSparkMax(Constants.Ports.LEFT_FRONT_PORT,MotorType.kBrushless);
        m_leftMiddleEngine  = new CANSparkMax(Constants.Ports.LEFT_MIDDLE_PORT,MotorType.kBrushless);
        m_leftBackEngine    = new CANSparkMax(Constants.Ports.LEFT_BACK_PORT,MotorType.kBrushless);
        m_rightFrontEngine  = new CANSparkMax(Constants.Ports.RIGHT_FRONT_PORT,MotorType.kBrushless);
        m_rightMiddleEngine = new CANSparkMax(Constants.Ports.RIGHT_MIDDLE_PORT,MotorType.kBrushless);
        m_rightBackEngine   = new CANSparkMax(Constants.Ports.RIGHT_BACK_PORT,MotorType.kBrushless);
        
        rightMotors = new MotorControllerGroup( m_rightBackEngine,m_rightFrontEngine,m_rightMiddleEngine);
        leftMotors  = new MotorControllerGroup( m_leftBackEngine,m_leftFrontEngine, m_leftMiddleEngine);
        rightMotors.setInverted(true);
    }

    
    public static Chassis getInstance(){
        if(m_instance==null){
            m_instance=new Chassis();
        }
        return m_instance;
    }

    public void driveTank(double lSpeed, double rSpeed){
        rightMotors.set(rSpeed);
        leftMotors.set(lSpeed);
    }

    public void turnRight(double speed){
       driveTank(speed, -speed);
    }

    public void turnLeft(double speed){
        driveTank(-speed, speed);
    }

    public void driveStraight(double speed){
        driveTank(speed, speed);
    }

    public void stop(){
        driveTank(0.0, 0.0);
    }
    
    public MotorControllerGroup getRightMotorControllerGroup(){
        return rightMotors;
    }
    public MotorControllerGroup getLeftMotorControllerGroup(){
        return leftMotors;
    }
    
}


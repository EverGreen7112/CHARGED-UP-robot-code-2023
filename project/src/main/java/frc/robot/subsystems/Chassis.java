package frc.robot.subsystems;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase{
    private MotorControllerGroup rightMotors;
    private MotorControllerGroup leftMotors;

    private MotorController m_leftFrontEngine, m_leftMiddleEngine, m_leftBackEngine;
    private MotorController m_rightFrontEngine, m_rightMiddleEngine, m_rightBackEngine;
    
    private static Chassis m_instance=null;

    public Chassis(){
        m_leftFrontEngine   = new Spark(Constants.Ports.LEFT_FRONT_PORT);
        m_leftMiddleEngine  = new Spark(Constants.Ports.LEFT_MIDDLE_PORT);
        m_leftBackEngine    = new Spark(Constants.Ports.LEFT_BACK_PORT);
        m_rightFrontEngine  = new Spark(Constants.Ports.RIGHT_FRONT_PORT);
        m_rightMiddleEngine = new Spark(Constants.Ports.RIGHT_MIDDLE_PORT);
        m_rightBackEngine   = new Spark(Constants.Ports.RIGHT_BACK_PORT);
        
        rightMotors = new MotorControllerGroup(m_rightFrontEngine, m_rightMiddleEngine, m_rightBackEngine);
        leftMotors = new MotorControllerGroup(m_leftFrontEngine, m_leftMiddleEngine, m_leftBackEngine);
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

    //TODO: implement later
    public double getRobotAngle() {
        return -1;
    }

    //TODO: implement later
    public double getEncodersDist() {
        return 0;
    }

    
    
}


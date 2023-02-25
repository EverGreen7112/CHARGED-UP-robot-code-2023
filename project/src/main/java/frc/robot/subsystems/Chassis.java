package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.SparkMaxPIDController;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDS;

public class Chassis extends SubsystemBase{
    private MotorControllerGroup rightMotors;
    private MotorControllerGroup leftMotors;

    
    //front are leaders
    private CANSparkMax m_leftFrontEngine, m_leftMiddleEngine, m_leftBackEngine;
    private CANSparkMax m_rightFrontEngine, m_rightMiddleEngine, m_rightBackEngine;

    private static Chassis m_instance=null;

    public Chassis(){
        m_leftFrontEngine   = new CANSparkMax(Constants.Ports.LEFT_FRONT_PORT,MotorType.kBrushless);
        m_leftFrontEngine   = new CANSparkMax(Constants.Ports.LEFT_FRONT_PORT, MotorType.kBrushless);
        m_leftMiddleEngine  = new CANSparkMax(Constants.Ports.LEFT_MIDDLE_PORT,MotorType.kBrushless);
        m_leftBackEngine    = new CANSparkMax(Constants.Ports.LEFT_BACK_PORT,MotorType.kBrushless);
        m_rightFrontEngine  = new CANSparkMax(Constants.Ports.RIGHT_FRONT_PORT,MotorType.kBrushless);
        m_rightMiddleEngine = new CANSparkMax(Constants.Ports.RIGHT_MIDDLE_PORT,MotorType.kBrushless);
        m_rightBackEngine   = new CANSparkMax(Constants.Ports.RIGHT_BACK_PORT,MotorType.kBrushless);
        
     
        rightMotors.setInverted(true);


        m_leftMiddleEngine.follow(m_leftFrontEngine);
        m_leftBackEngine.follow(m_leftFrontEngine);
        m_rightMiddleEngine.follow(m_rightFrontEngine);
        m_rightBackEngine.follow(m_rightFrontEngine);

        m_rightFrontEngine.getPIDController().setP(PIDS.driveKp,0);
        m_rightFrontEngine.getPIDController().setD(PIDS.driveKd,0);
        m_rightFrontEngine.getPIDController().setI(PIDS.driveKi,0);
        m_rightFrontEngine.getPIDController().setP(PIDS.driveKp,0);
        m_rightFrontEngine.getPIDController().setD(PIDS.driveKd,0);
        m_rightFrontEngine.getPIDController().setI(PIDS.driveKi,0); 
        
        m_rightFrontEngine.getPIDController().setP(PIDS.velKp,1);
        m_rightFrontEngine.getPIDController().setD(PIDS.velKd,1);
        m_rightFrontEngine.getPIDController().setI(PIDS.velKi,1);
        m_rightFrontEngine.getPIDController().setP(PIDS.velKp,1);
        m_rightFrontEngine.getPIDController().setD(PIDS.velKd,1);
        m_rightFrontEngine.getPIDController().setI(PIDS.velKi,1);

        m_rightFrontEngine.getEncoder().setPositionConversionFactor(Constants.Values.DISTANCE_PER_THICK);

    }

    public static Chassis getInstance(){
        if(m_instance==null){
            m_instance=new Chassis();
        }
        return m_instance;
    }
    

    public void driveTank(double lSpeed, double rSpeed){
        m_rightFrontEngine.set(rSpeed);
        m_leftFrontEngine.set(lSpeed);
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
    public double getRobotAngle(){
        //TODO: implement later
        return -1;
    }
    public double getEncodersDist(){
        return (getRightEncoderDist()+getLeftEncoderDist())/2;
    }
    public double getRightEncoderDist(){
        return m_rightFrontEngine.getEncoder().getPosition();
    }
    public double getLeftEncoderDist(){
        return m_leftFrontEngine.getEncoder().getPosition();
    }
    public double getVelocity(){
         return (getRightVelocity()+getLeftVelocity())/2;
    }
    public double getRightVelocity(){
        return m_rightFrontEngine.getEncoder().getVelocity();
    }
    public double getLeftVelocity(){
        return m_leftFrontEngine.getEncoder().getVelocity();
    }
    
    public SparkMaxPIDController getRightPID(){
        return m_rightFrontEngine.getPIDController();
    }
    public SparkMaxPIDController getLeftPID(){
        return m_leftFrontEngine.getPIDController();
    }
    
    public MotorControllerGroup getRightMotorControllerGroup(){
        return rightMotors;
    }
    public MotorControllerGroup getLeftMotorControllerGroup(){
        return leftMotors;
    }
    
}

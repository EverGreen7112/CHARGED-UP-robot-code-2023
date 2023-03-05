package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vector2D;
import frc.robot.Vision;
import frc.robot.Constants.PIDS;

public class Chassis extends SubsystemBase {

    private static AHRS m_navx;

    // front are leaders
    public static CANSparkMax m_leftFrontEngine, m_leftMiddleEngine, m_leftBackEngine;
    public static CANSparkMax m_rightFrontEngine, m_rightMiddleEngine, m_rightBackEngine;
    private static double m_time;
    private static double m_deltaTime;
    private static double m_lastRightDist;
    private static double m_lastLeftDist;
    private static Vector2D m_pos;
    private static Vision m_vision;

    private static Chassis m_instance = null;

    public Chassis() {
        m_leftFrontEngine = new CANSparkMax(Constants.Ports.LEFT_FRONT_PORT, MotorType.kBrushless);
        m_leftMiddleEngine = new CANSparkMax(Constants.Ports.LEFT_MIDDLE_PORT, MotorType.kBrushless);
        m_leftBackEngine = new CANSparkMax(Constants.Ports.LEFT_BACK_PORT, MotorType.kBrushless);
        m_rightFrontEngine = new CANSparkMax(Constants.Ports.RIGHT_FRONT_PORT, MotorType.kBrushless);
       m_rightMiddleEngine = new CANSparkMax(Constants.Ports.RIGHT_MIDDLE_PORT, MotorType.kBrushless);
        m_rightBackEngine = new CANSparkMax(Constants.Ports.RIGHT_BACK_PORT, MotorType.kBrushless);

      


        m_leftMiddleEngine.follow(m_leftFrontEngine);
        m_leftBackEngine.follow(m_leftFrontEngine);
        m_rightMiddleEngine.follow(m_rightFrontEngine);
        m_rightBackEngine.follow(m_rightFrontEngine);

        m_rightFrontEngine.getPIDController().setP(PIDS.driveKp, 0);
        m_rightFrontEngine.getPIDController().setD(PIDS.driveKd, 0);
        m_rightFrontEngine.getPIDController().setI(PIDS.driveKi, 0);
        m_rightFrontEngine.getPIDController().setP(PIDS.driveKp, 0);
        m_rightFrontEngine.getPIDController().setD(PIDS.driveKd, 0);
        m_rightFrontEngine.getPIDController().setI(PIDS.driveKi, 0);

        m_rightFrontEngine.getPIDController().setP(PIDS.velKp, 1);
        m_rightFrontEngine.getPIDController().setD(PIDS.velKd, 1);
        m_rightFrontEngine.getPIDController().setI(PIDS.velKi, 1);
        m_rightFrontEngine.getPIDController().setP(PIDS.velKp, 1);
        m_rightFrontEngine.getPIDController().setD(PIDS.velKd, 1);
        m_rightFrontEngine.getPIDController().setI(PIDS.velKi, 1);

        m_leftFrontEngine.getEncoder().setPositionConversionFactor(Constants.Values.DISTANCE_PER_TICK);
        m_rightFrontEngine.getEncoder().setPositionConversionFactor(Constants.Values.DISTANCE_PER_TICK);

        m_leftFrontEngine.getEncoder().setPosition(0);
        m_rightFrontEngine.getEncoder().setPosition(0);
        m_time = System.currentTimeMillis();
        m_lastRightDist = m_rightFrontEngine.getEncoder().getPosition();
        m_lastLeftDist = m_leftFrontEngine.getEncoder().getPosition();
        m_pos = new Vector2D(0,0);
        m_navx = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

        m_deltaTime = 0;

        m_vision = new Vision(5800);
    }

    @Override
    public void periodic() {

        // double deltaRight = Chassis.getRightEncoderDist() - m_lastRightDist;
        // m_lastRightDist = Chassis.getRightEncoderDist();

        // double deltaLeft = Chassis.getRightEncoderDist() - m_lastLeftDist;
        // m_lastLeftDist = Chassis.getLeftEncoderDist();

        // double leftSideAngle = deltaLeft / Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;
        // double rightSideAngle = deltaRight / Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;

        // Vector2D left = new Vector2D(
        //         Math.cos(0.5 * leftSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT,
        //         2 * Math.sin(0.5 * leftSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT);

        // Vector2D right = new Vector2D(
        //         Math.cos(0.5 * rightSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT,
        //         2 * Math.sin(0.5 *rightSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT);

        // Vector2D robotDelta = right.getAdded(left).getDivided(2);
        // Vector2D fieldDelta = Constants.Conversions.rotateZ(robotDelta, -Chassis.getRobotAngle());
        // m_pos.add(fieldDelta);
        // SmartDashboard.putNumber("x", m_pos.x);
        // SmartDashboard.putNumber("y", m_pos.y);
        // SmartDashboard.putNumber("RPM left velocity",
        // Chassis.getLeftVelocity() / 10.97);
        // SmartDashboard.putNumber("RPM right velocity",
        // Chassis.getRightVelocity() / 10.97);
        // SmartDashboard.putNumber("ms left velocity",
        // Constants.Conversions.rpm2ms(Constants.Values.TANKDRIVE_WHEEL_RADIUS,
        // Chassis.getLeftVelocity()/ 10.97));
        // SmartDashboard.putNumber("ms right velocity",
        // Constants.Conversions.rpm2ms(Constants.Values.TANKDRIVE_WHEEL_RADIUS,
        // Chassis.getRightVelocity()/ 10.97));
        // SmartDashboard.putNumber("left Angle", Math.toDegrees(leftSideAngle));
        // SmartDashboard.putNumber("right Angle", Math.toDegrees(rightSideAngle));
        // SmartDashboard.putNumber("leftSideDistance", this.getLeftEncoderDist());
        // SmartDashboard.putNumber("rightSideDistance", this.getRightEncoderDist());
        
        // SmartDashboard.putNumber("x", m_vision.getX());
        // SmartDashboard.putNumber("y", m_vision.getY());
        // SmartDashboard.putNumber("z", m_vision.getZ());
        
      //  SmartDashboard.putNumber("distanceR",Chassis.getRightEncoderDist());
      //  SmartDashboard.putNumber("distanceL",Chassis.getLeftEncoderDist());
    }

    public static Chassis getInstance() {
        if (m_instance == null) {
            m_instance = new Chassis();
        }
        return m_instance;
    }

    public static void driveTank(double lSpeed, double rSpeed) {
        m_rightFrontEngine.set(rSpeed);
        m_leftFrontEngine.set(lSpeed);
    }

    public static void turnRight(double speed) {
        driveTank(speed, -speed);
    }

    public static void turnLeft(double speed) {
        driveTank(-speed, speed);
    }

    public static void driveStraight(double speed) {
        driveTank(speed, speed);
    }

    public static void stop() {
        driveTank(0.0, 0.0);
    }

    public static double getRobotAngle() {
        return m_navx.getAngle();
    }

    public static double getEncodersDist() {
        return (getRightEncoderDist() + getLeftEncoderDist()) / 2;
    }

    public static double getRightEncoderDist() {
        // return -1 * m_rightFrontEngine.getEncoder().getPosition() / 10.97;
        return -1 * m_rightFrontEngine.getEncoder().getPosition();
    }

    public static double getLeftEncoderDist() {
        // return m_leftFrontEngine.getEncoder().getPosition() / 10.97;
        return m_leftFrontEngine.getEncoder().getPosition();
    }

    public static double getVelocity() {
        return (getRightVelocity() + getLeftVelocity()) / 2;
    }

    public static double getRightVelocity() {
        return m_rightFrontEngine.getEncoder().getVelocity();
    }

    public static double getLeftVelocity() {
        return m_leftFrontEngine.getEncoder().getVelocity();
    }

    public static SparkMaxPIDController getRightPID() {
        return m_rightFrontEngine.getPIDController();
    }

    public static SparkMaxPIDController getLeftPID() {
        return m_leftFrontEngine.getPIDController();
    }

   


}

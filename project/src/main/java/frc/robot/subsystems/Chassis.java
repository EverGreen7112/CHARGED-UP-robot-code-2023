package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vector2D;
import frc.robot.Vision;
import frc.robot.Constants.PIDS;
import frc.robot.commands.DriveDistanceByEncoders;
import frc.robot.commands.TurnToAnglePID;

public class Chassis extends SubsystemBase {
    private static MotorControllerGroup rightMotors;
    private static MotorControllerGroup leftMotors;
    private static AHRS m_navx;

    // front are leaders
    public static CANSparkMax m_leftFrontEngine, m_leftMiddleEngine, m_leftBackEngine;
    public static CANSparkMax m_rightFrontEngine, m_rightMiddleEngine, m_rightBackEngine;

    private static Vision m_robotLocation, m_reflector;

    private static Chassis m_instance = null;

    public Chassis() {
        m_leftFrontEngine = new CANSparkMax(Constants.Ports.LEFT_FRONT_PORT, MotorType.kBrushless);
        m_leftMiddleEngine = new CANSparkMax(Constants.Ports.LEFT_MIDDLE_PORT, MotorType.kBrushless);
        m_leftBackEngine = new CANSparkMax(Constants.Ports.LEFT_BACK_PORT, MotorType.kBrushless);
        m_rightFrontEngine = new CANSparkMax(Constants.Ports.RIGHT_FRONT_PORT, MotorType.kBrushless);
        m_rightMiddleEngine = new CANSparkMax(Constants.Ports.RIGHT_MIDDLE_PORT, MotorType.kBrushless);
        m_rightBackEngine = new CANSparkMax(Constants.Ports.RIGHT_BACK_PORT, MotorType.kBrushless);

        leftMotors = new MotorControllerGroup(m_leftFrontEngine, m_leftMiddleEngine, m_leftBackEngine);
        rightMotors = new MotorControllerGroup(m_rightFrontEngine, m_rightMiddleEngine, m_rightBackEngine);

        leftMotors.setInverted(true);




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
 
        m_navx = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

        m_robotLocation = new Vision(Constants.Ports.RECIEVE_LOCATION_PORT);
        m_reflector = new Vision(Constants.Ports.REFLECTOR_PORT);
    }

    public static AHRS getGyro(){
        return getInstance().m_navx;
    }
    public static void test(){
        m_leftBackEngine.set(0.4);
    }
    @Override
    public void periodic() {

        // double deltaRight = Chassis.getInstance().getRightEncoderDist() - m_lastRightDist;
        // m_lastRightDist = Chassis.getInstance().getRightEncoderDist();

        // double deltaLeft = Chassis.getInstance().getRightEncoderDist() - m_lastLeftDist;
        // m_lastLeftDist = Chassis.getInstance().getLeftEncoderDist();

        // double leftSideAngle = deltaLeft / Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;
        // double rightSideAngle = deltaRight / Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;

        // Vector2D left = new Vector2D(
        //         Math.cos(0.5 * leftSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT,
        //         2 * Math.sin(0.5 * leftSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT);

        // Vector2D right = new Vector2D(
        //         Math.cos(0.5 * rightSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT,
        //         2 * Math.sin(0.5 *rightSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT);

        // Vector2D robotDelta = right.getAdded(left).getDivided(2);
        // Vector2D fieldDelta = Constants.Conversions.rotateZ(robotDelta, -Chassis.getInstance().getRobotAngle());
        // m_pos.add(fieldDelta);
        // SmartDashboard.putNumber("x", m_pos.x);
        // SmartDashboard.putNumber("y", m_pos.y);
        // SmartDashboard.putNumber("RPM left velocity",
        // Chassis.getInstance().getLeftVelocity() / 10.97);
        // SmartDashboard.putNumber("RPM right velocity",
        // Chassis.getInstance().getRightVelocity() / 10.97);
        // SmartDashboard.putNumber("ms left velocity",
        // Constants.Conversions.rpm2ms(Constants.Values.TANKDRIVE_WHEEL_RADIUS,
        // Chassis.getInstance().getLeftVelocity()/ 10.97));
        // SmartDashboard.putNumber("ms right velocity",
        // Constants.Conversions.rpm2ms(Constants.Values.TANKDRIVE_WHEEL_RADIUS,
        // Chassis.getInstance().getRightVelocity()/ 10.97));
        // SmartDashboard.putNumber("left Angle", Math.toDegrees(leftSideAngle));
        // SmartDashboard.putNumber("right Angle", Math.toDegrees(rightSideAngle));
        // SmartDashboard.putNumber("leftSideDistance", this.getLeftEncoderDist());
        // SmartDashboard.putNumber("rightSideDistance", this.getRightEncoderDist());
        
        // SmartDashboard.putNumber("x", m_vision.getX());
        // SmartDashboard.putNumber("y", m_vision.getY());
        // SmartDashboard.putNumber("z", m_vision.getZ());
        
      //  SmartDashboard.putNumber("distanceR",Chassis.getInstance().getRightEncoderDist());
      //  SmartDashboard.putNumber("distanceL",Chassis.getInstance().getLeftEncoderDist());
    }

    public static Chassis getInstance() {
        if (m_instance == null) {
            m_instance = new Chassis();
        }
        return m_instance;
    }

    public static void driveTank(double lSpeed, double rSpeed) {
        rightMotors.set(rSpeed);
        leftMotors.set(lSpeed);
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

    public static double getSpeedMagnitude(){
        return Math.sqrt(leftMotors.get() * leftMotors.get() + rightMotors.get() * rightMotors.get());
    }
    public static float[] getRobotLocation() {
        return m_robotLocation.getXYZ();
    }

    public static double getRobotAngle() {
        return m_navx.getYaw();
    }

    public static double getEncodersDist() {
        return -(getRightEncoderDist() + getLeftEncoderDist()) / 2;
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

    public static  SparkMaxPIDController getLeftPID() {
        return m_leftFrontEngine.getPIDController();
    }

    public static  MotorControllerGroup getRightMotorControllerGroup() {
        return rightMotors;
    }

    public static MotorControllerGroup getLeftMotorControllerGroup() {
        return leftMotors;
    }
    public static void resetGyro(){
        m_navx.reset();
    }

    public static double getAngleToReflector() {
        return Math.atan2(m_reflector.getX(), m_reflector.getZ()) * 180 / Math.PI;
    }
   

    public double calcTargetX(){
        return m_reflector.getX() + Constants.Values.X_AXIS_OFFSET * -Math.signum(m_robotLocation.getX());
    }
    public double calcTargetZ(){
        return m_reflector.getZ();
    }
    
    public void getToPos(double angle, double magnitude){
        new SequentialCommandGroup(new TurnToAnglePID(angle), new DriveDistanceByEncoders(magnitude, 0.1), new TurnToAnglePID(0)).schedule();
    }
    public void driveToReflactor(){
        Vector2D target = new Vector2D(calcTargetX(), calcTargetZ());
        getToPos(target.getAngle(), target.getLength());
    }


    public static void setMode(IdleMode mode){
        m_leftBackEngine.setIdleMode(mode);
        m_leftMiddleEngine.setIdleMode(mode);
        m_leftFrontEngine.setIdleMode(mode);
        m_rightBackEngine.setIdleMode(mode);
        m_rightMiddleEngine.setIdleMode(mode);
        m_rightFrontEngine.setIdleMode(mode);
    }

    public static IdleMode getMode(){
        return m_leftBackEngine.getIdleMode();
    }
}
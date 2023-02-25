package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vector2D;
import frc.robot.Constants.PIDS;

public class Chassis extends SubsystemBase {
    private MotorControllerGroup rightMotors;
    private MotorControllerGroup leftMotors;

    // front are leaders
    private CANSparkMax m_leftFrontEngine, m_leftMiddleEngine, m_leftBackEngine;
    private CANSparkMax m_rightFrontEngine, m_rightMiddleEngine, m_rightBackEngine;
    private double m_time;
    private double m_lastRightDist;
    private double m_lastLeftDist;
    private Vector2D m_pos;

    private static Chassis m_instance = null;

    public Chassis() {
        m_leftFrontEngine = new CANSparkMax(Constants.Ports.LEFT_FRONT_PORT, MotorType.kBrushless);
        m_leftFrontEngine = new CANSparkMax(Constants.Ports.LEFT_FRONT_PORT, MotorType.kBrushless);
        m_leftMiddleEngine = new CANSparkMax(Constants.Ports.LEFT_MIDDLE_PORT, MotorType.kBrushless);
        m_leftBackEngine = new CANSparkMax(Constants.Ports.LEFT_BACK_PORT, MotorType.kBrushless);
        m_rightFrontEngine = new CANSparkMax(Constants.Ports.RIGHT_FRONT_PORT, MotorType.kBrushless);
        m_rightMiddleEngine = new CANSparkMax(Constants.Ports.RIGHT_MIDDLE_PORT, MotorType.kBrushless);
        m_rightBackEngine = new CANSparkMax(Constants.Ports.RIGHT_BACK_PORT, MotorType.kBrushless);

        rightMotors.setInverted(true);

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

        m_rightFrontEngine.getEncoder().setPositionConversionFactor(Constants.Values.DISTANCE_PER_THICK);
        m_time = System.currentTimeMillis();
        m_lastRightDist = m_rightFrontEngine.getEncoder().getPosition();
        m_lastLeftDist = m_leftFrontEngine.getEncoder().getPosition();
        m_pos = new Vector2D(0,0);


    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("robot speed", 0.4);


        double deltaRight = Chassis.getInstance().getRightEncoderDist() - m_lastRightDist;
        m_lastRightDist = Chassis.getInstance().getRightEncoderDist();

        double deltaLeft = Chassis.getInstance().getRightEncoderDist() - m_lastLeftDist;
        m_lastLeftDist = Chassis.getInstance().getLeftEncoderDist();

        double leftSideAngle = deltaLeft / Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;
        double rightSideAngle = deltaRight / Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;

        Vector2D left = new Vector2D(
                Math.cos(leftSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT,
                Math.sin(leftSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT);

        Vector2D right = new Vector2D(Math.cos(rightSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT,
                Math.sin(rightSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT);

        Vector2D robotDelta = right.getAdded(left).getDivided(2);
        Vector2D fieldDelta = Constants.Conversions.rotateZ(robotDelta, -Chassis.getInstance().getRobotAngle());
        m_pos.add(fieldDelta);
        SmartDashboard.putNumber("x", m_pos.x);
        SmartDashboard.putNumber("y", m_pos.y);
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
        // SmartDashboard.putNumber("leftSideDistance", m_leftSideDistance);
        // SmartDashboard.putNumber("rightSideDistance", m_rightSideDistance);
        // SmartDashboard.putNumber("test", Math.sqrt(leftSideX * leftSideX + leftSideY
        // * leftSideY));
        // SmartDashboard.putNumber("test2", Math.sqrt(rightSideX * rightSideX +
        // rightSideY * rightSideX));

    }

    public static Chassis getInstance() {
        if (m_instance == null) {
            m_instance = new Chassis();
        }
        return m_instance;
    }

    public void driveTank(double lSpeed, double rSpeed) {
        m_rightFrontEngine.set(rSpeed);
        m_leftFrontEngine.set(lSpeed);
    }

    public void turnRight(double speed) {
        driveTank(speed, -speed);
    }

    public void turnLeft(double speed) {
        driveTank(-speed, speed);
    }

    public void driveStraight(double speed) {
        driveTank(speed, speed);
    }

    public void stop() {
        driveTank(0.0, 0.0);
    }

    public double getRobotAngle() {
        // TODO: implement later
        return -1;
    }

    public double getEncodersDist() {
        return (getRightEncoderDist() + getLeftEncoderDist()) / 2;
    }

    public double getRightEncoderDist() {
        return m_rightFrontEngine.getEncoder().getPosition();
    }

    public double getLeftEncoderDist() {
        return m_leftFrontEngine.getEncoder().getPosition();
    }

    public double getVelocity() {
        return (getRightVelocity() + getLeftVelocity()) / 2;
    }

    public double getRightVelocity() {
        return m_rightFrontEngine.getEncoder().getVelocity();
    }

    public double getLeftVelocity() {
        return m_leftFrontEngine.getEncoder().getVelocity();
    }

    public SparkMaxPIDController getRightPID() {
        return m_rightFrontEngine.getPIDController();
    }

    public SparkMaxPIDController getLeftPID() {
        return m_leftFrontEngine.getPIDController();
    }

    public MotorControllerGroup getRightMotorControllerGroup() {
        return rightMotors;
    }

    public MotorControllerGroup getLeftMotorControllerGroup() {
        return leftMotors;
    }

}

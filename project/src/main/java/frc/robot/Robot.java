
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Chassis;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private long m_time;
  private double m_deltaTime;
  private double m_lastRightDist;
  private double m_lastLeftDist;

  private AHRS m_navx = new AHRS(SPI.Port.kMXP);
  private Vector2D m_pos = new Vector2D(0,0);

  private double m_rightSideDistance = 0;
  private double m_leftSideDistance = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_time = System.currentTimeMillis();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("robot speed", 0.4);
  
    m_deltaTime = System.currentTimeMillis() / 1000.0 - m_time / 1000.0;

    double deltaRight = Chassis.getInstance().getRightEncoderDist()-m_lastRightDist;
    m_lastRightDist = Chassis.getInstance().getRightEncoderDist();

    double deltaLeft = Chassis.getInstance().getRightEncoderDist()-m_lastLeftDist;
    m_lastLeftDist = Chassis.getInstance().getLeftEncoderDist();

    double leftSideAngle = deltaLeft / Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;
    double rightSideAngle = deltaRight / Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;
    
   Vector2D left = new Vector2D(
    Math.cos(leftSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT
    ,Math.sin(leftSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT);
    
    Vector2D right = new Vector2D(Math.cos(rightSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT,
    Math.sin(rightSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT);
    
    Vector2D robotDelta = right.getAdded(left).getDivided(2);
    Vector2D fieldDelta = Constants.Conversions.rotateZ(robotDelta,-Chassis.getInstance().getRobotAngle());
    m_pos.add(fieldDelta);
    SmartDashboard.putNumber("x", m_pos.x);
    SmartDashboard.putNumber("y", m_pos.y);
    // SmartDashboard.putNumber("RPM left velocity",  Chassis.getInstance().getLeftVelocity() / 10.97);
    // SmartDashboard.putNumber("RPM right velocity", Chassis.getInstance().getRightVelocity() / 10.97);
    // SmartDashboard.putNumber("ms left velocity", Constants.Conversions.rpm2ms(Constants.Values.TANKDRIVE_WHEEL_RADIUS, Chassis.getInstance().getLeftVelocity()/ 10.97));
    // SmartDashboard.putNumber("ms right velocity", Constants.Conversions.rpm2ms(Constants.Values.TANKDRIVE_WHEEL_RADIUS, Chassis.getInstance().getRightVelocity()/ 10.97));
    // SmartDashboard.putNumber("left Angle", Math.toDegrees(leftSideAngle));
    // SmartDashboard.putNumber("right Angle", Math.toDegrees(rightSideAngle));
    // SmartDashboard.putNumber("leftSideDistance", m_leftSideDistance);
    // SmartDashboard.putNumber("rightSideDistance", m_rightSideDistance);
    // SmartDashboard.putNumber("test", Math.sqrt(leftSideX * leftSideX  + leftSideY * leftSideY));
    // SmartDashboard.putNumber("test2", Math.sqrt(rightSideX * rightSideX + rightSideY * rightSideX));  
    m_time = System.currentTimeMillis();
  
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().schedule(RobotContainer.m_tankDriveCommand);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // b.set(ControlMode.PercentOutput, -0.1);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
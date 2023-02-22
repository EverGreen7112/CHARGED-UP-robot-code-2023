
package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TankDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private long m_time;
  private double m_deltaTime; 
  private CANSparkMax m_frontRightWheel = new CANSparkMax(Constants.Ports.RIGHT_FRONT_PORT, MotorType.kBrushless);
  private CANSparkMax m_frontLeftWheel  = new CANSparkMax(Constants.Ports.LEFT_FRONT_PORT,  MotorType.kBrushless);
  private AHRS m_navx = new AHRS(SPI.Port.kMXP);
  private double m_xPos = 0;
  private double m_yPos = 0;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  
    m_deltaTime = System.currentTimeMillis() / 1000.0 - m_time / 1000.0;

    double leftSideVelocity = Constants.Conversions.rpm2ms(Constants.Values.TANKDRIVE_WHEEL_RADIUS, m_frontLeftWheel.getEncoder().getVelocity());
    double rightSideVelocity = Constants.Conversions.rpm2ms(Constants.Values.TANKDRIVE_WHEEL_RADIUS, m_frontRightWheel.getEncoder().getVelocity());

    double leftSideDistance = leftSideVelocity * m_deltaTime;
    double rightSideDistance = rightSideVelocity * m_deltaTime; 
   
    double leftSideAngle = leftSideDistance / Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;
    double rightSideAngle = rightSideDistance / Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;
    
    double leftSideX  = Math.cos(leftSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;
    double leftSideY  = Math.sin(leftSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;
    
    double rightSideX = Math.cos(rightSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;
    double rightSideY = Math.sin(rightSideAngle) * Constants.Values.DISTANCE_BETWEEN_LEFT_TO_RIGHT;

    double deltaX = leftSideX + rightSideX;
    double deltaY = leftSideY + rightSideY;

    m_xPos += deltaX;
    m_yPos += deltaY;

    SmartDashboard.putNumber("x", m_xPos);
    SmartDashboard.putNumber("y", m_yPos);
    m_time = System.currentTimeMillis();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
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
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    RobotContainer.m_tankDriveCommand.execute();
    
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.unused.DriveDistanceByEncoders;
import frc.robot.commands.unused.JoyStickSum;
import frc.robot.subsystems.Arm;
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
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.run();
    SmartDashboard.putNumber("First Angle", Constants.Conversions.ticksToAngle(Arm.getFirst().getSelectedSensorPosition(), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION));
    SmartDashboard.putNumber("Second Angle", Constants.Conversions.ticksToAngle(Arm.getSecond().getSelectedSensorPosition(), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION));
    //SmartDashboard.putNumber("distance", Constants.Conversions.ticksToMeters(Chassis.getEncodersDist(), Constants.Values.DISTANCE_PER_TICK)); 
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */

  @Override
  public void autonomousInit() {
    CommandScheduler.cancelAll();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Chassis.m_leftFrontEngine.getEncoder().setPosition(0);
    Chassis.m_rightFrontEngine.getEncoder().setPosition(0);
    Chassis.m_leftMiddleEngine.getEncoder().setPosition(0);
    Chassis.m_rightMiddleEngine.getEncoder().setPosition(0);
    Chassis.m_leftBackEngine.getEncoder().setPosition(0);
    Chassis.m_rightBackEngine.getEncoder().setPosition(0);
    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    (new DriveDistanceByEncoders(3, 0.01, 0.01)).schedule();
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
    CommandScheduler.cancelAll();
    Arm.getFirst().set(TalonFXControlMode.PercentOutput , 0);
    Arm.getSecond().set(TalonFXControlMode.PercentOutput , 0);
    
    // CommandScheduler.schedule(new SetArmAngleToStartPos());
    RobotContainer.m_tankDriveCommand.schedule();
  }

  JoyStickSum j = new JoyStickSum();
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // j.schedule();
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.cancelAll();
    (new DriveDistanceByEncoders(1.2, 0.1)).schedule();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    CommandScheduler.run();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
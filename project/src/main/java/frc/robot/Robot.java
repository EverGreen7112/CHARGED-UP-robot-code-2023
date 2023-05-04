
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.ArmMoveAndStayAtAngle;
import frc.robot.commands.General.Commands;
import frc.robot.commands.Gripper.CloseGripper;
import frc.robot.commands.Gripper.GripCube;
import frc.robot.commands.Gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gripper;

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
  private boolean m_placeCubeAndBalanceAuto, m_drive, m_balance, m_placeConeAuto;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /*
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("slow", SmartDashboard.getNumber("slow", 0.2));
    SmartDashboard.putNumber("normal", SmartDashboard.getNumber("normal", 0.6));
    SmartDashboard.putNumber("Turbo", SmartDashboard.getNumber("Turbo", Constants.Speeds.TURBO));
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5800, "limelight.local", 5800);
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Chassis.getInstance();
    Arm.getInstance();
    Gripper.getInstance();
    Arm.getFirst().getEncoder().setPosition(0);
    Arm.getSecond().getEncoder().setPosition(0);
    m_chooser.setDefaultOption("placeGamePieceAuto", Commands.placeGamePieceAuto);
    m_chooser.addOption("placeGamePieceAuto", Commands.placeGamePieceAuto);
    m_chooser.addOption("placeGamePieceAndBalanceAuto", Commands.placeGamePieceAndBalanceAuto);
    m_chooser.addOption("driveOnlyAuto", Commands.driveOnlyAuto);
    m_chooser.addOption("balanceOnlyAuto", Commands.balanceOnlyAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void disabledExit() {
    Arm.getInstance();
    Arm.getFirst().setIdleMode(IdleMode.kBrake);
    Arm.getSecond().setIdleMode(IdleMode.kBrake);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    SmartDashboard.putNumber("First Angle", Arm.getFirstAngle());
    SmartDashboard.putNumber("Second Angle", Arm.getSecondAngle());
    SmartDashboard.putString("chassis mode", Chassis.getMode().name());
  }
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    Arm.getInstance();
    Arm.getFirst().setIdleMode(IdleMode.kCoast);
    Arm.getSecond().setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */

  ArmMoveAndStayAtAngle moveAndStay;

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    Chassis.getGyro().reset();
    
    m_chooser.getSelected().schedule();
}

    // new DriveDistanceByEncoders(-1, 0.05, 0.05).schedule();.

  @Override
  public void autonomousPeriodic() {}
  
  // ArmMoveAndStayAtAngle manualMove = new ArmMoveAndStayAtAngle(0, 0, 1, false);
  @Override
  public void teleopInit() {
    Chassis.resetGyro();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().cancelAll();
    Chassis.setMode(IdleMode.kCoast);

    RobotContainer.m_tankDriveCommand.schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}
  @Override
  public void testInit() {
    Trigger closeGripper = new JoystickButton(RobotContainer.m_operator, Constants.ButtonPorts.Y).onTrue(new CloseGripper());
    Trigger openGripper = new JoystickButton(RobotContainer.m_operator, Constants.ButtonPorts.B).onTrue(new OpenGripper());
    Trigger closeToCube = new JoystickButton(RobotContainer.m_operator, Constants.ButtonPorts.X).onTrue(new GripCube());
    Trigger tightenGrip = new JoystickButton(RobotContainer.m_operator, Constants.ButtonPorts.A).whileTrue(Commands.tightenGrip);

  }

  /** This function is called periodically during test mode. */
  // CANSparkMax _test = new CANSparkMax(14, MotorType.kBrushless);

  @Override
  public void testPeriodic() {  }

  /** This functionv is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
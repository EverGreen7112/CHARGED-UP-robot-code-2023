// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.General.Commands;
import frc.robot.commands.Gripper.CloseGripper;
import frc.robot.commands.Gripper.GripCube;
import frc.robot.commands.Gripper.OpenGripper;
// import frc.robot.commands.ArmTwoStayInZero;
// import frc.robot.commands.AutoMove;
// import frc.robot.commands.CloseGripper;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
// import frc.robot.commands.MotionMagicArmPID;
// import frc.robot.commands.GripCube;
// import frc.robot.commands.MoveArmByAngle;
// import frc.robot.commands.MoveSecStart;
// import frc.robot.commands.MoveArmByAngleSupllier;
// import frc.robot.commands.OpenGripper;
// import frc.robot.commands.SetArmAngleToStartPos;
// import frc.robot.commands.Slow;
// import frc.robot.commands.Turbo;
// import frc.robot.commands.TankDrive;
// import frc.robot.commands.TurnArmTwo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Gripper;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very lit\tle robot logic should actually be handled
 * in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // public static final Vision REFLECTOR = new Vision(Constants.Ports.REFLECTOR_PORT);
  // public static final Vision ROBOT_LOCATION = new Vision(Constants.Ports.RECIEVE_LOCATION_PORT);
  public static final Joystick m_leftStick = new Joystick(Constants.JoystickPorts.leftJoystick);
  public static final Joystick m_rightStick = new Joystick(Constants.JoystickPorts.rightJoystick);
  public static final Joystick m_operator = new Joystick(Constants.JoystickPorts.operator);
  // public static TankDrive m_tankDriveCommand = new TankDrive(m_leftStick::getY, m_rightStick::getY);
  public static Trigger up = new POVButton(m_operator, 0);
  public static Trigger down = new POVButton(m_operator, 180);
  public static Trigger lefter = new POVButton(m_operator, 270);
  public static Trigger righter = new POVButton(m_operator, 90);

 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Trigger rtButton = new JoystickButton(m_operator, 12).onTrue(new MotionMagicArmPID());
    Trigger closeGripper = new JoystickButton(m_operator, Constants.ButtonPorts.Y).onTrue(new CloseGripper());
    Trigger openGripper = new JoystickButton(m_operator, Constants.ButtonPorts.B).onTrue(new OpenGripper());
    Trigger closeToCube = new JoystickButton(m_operator, Constants.ButtonPorts.X).onTrue(new GripCube());
  //  Trigger armTwoToZero = new JoystickButton(m_operator, Constants.ButtonPorts.A).onTrue(new ArmTwoStayInZero());

    Trigger balance = new JoystickButton(m_operator, Constants.ButtonPorts.RB).whileTrue(new Balance());
    //Trigger openArmToCollectCone = new JoystickButton(m_operator, Constants.ButtonPorts.START).onTrue(new MoveArmByAngle(120, -55));
    //Trigger openArmToCollectCube = new JoystickButton(m_operator, Constants.ButtonPorts.BACK).onTrue(new MoveArmByAngle(105.49851545607983, -61.848));

    Trigger smallArmPlus = new JoystickButton(m_operator, Constants.ButtonPorts.RB).whileTrue(Commands.upperSmall);
    Trigger smallArmMinus = new JoystickButton(m_operator, Constants.ButtonPorts.RT).whileTrue(Commands.lowerSmall);

    
    Trigger bigArmPlus = new JoystickButton(m_operator, Constants.ButtonPorts.LB).whileTrue(Commands.upperBig);
    Trigger bigArmMinus = new JoystickButton(m_operator, Constants.ButtonPorts.LT).whileTrue(Commands.lowerBig);

    // Trigger turbo = new JoystickButton(m_leftStick, 1).whileTrue(new Turbo(m_tankDriveCommand));
    // Trigger slow = new JoystickButton(m_rightStick, 2).whileTrue(new Slow(m_tankDriveCommand));

    righter.onTrue(Commands.toggleConeIn);
    Trigger trig = new Trigger(Commands::joystick1OutOfRange);
    Trigger trig2 = new Trigger(Commands::joystic2OutOfRange);
    // RobotContainer.up.onTrue(new MoveArmByAngle(-128,160));
    // RobotContainer.down.onTrue(new MoveArmByAngle(-125, -288+360));

    Trigger a = new JoystickButton(m_rightStick, 4).onTrue(Commands.invertChassis);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}

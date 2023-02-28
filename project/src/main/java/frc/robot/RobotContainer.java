// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.GripCube;
import frc.robot.commands.MoveArmByAngle;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.SetArmAngleToStartPos;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very lit\tle robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public static final Joystick m_leftStick = new Joystick(Constants.JoystickPorts.leftJoystick);
  public static final Joystick m_rightStick = new Joystick(Constants.JoystickPorts.rightJoystick);
  public static final Joystick m_operator = new Joystick(Constants.JoystickPorts.operator);
  public static Command m_tankDriveCommand = new TankDrive(m_leftStick::getY,m_rightStick::getY);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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
    Trigger closeButton = new JoystickButton(m_operator, Constants.ButtonPorts.Y).onTrue(new CloseGripper());
    Trigger openGrip = new JoystickButton(m_operator, Constants.ButtonPorts.B).onTrue(new OpenGripper());

    // Trigger rtButton = new JoystickButton(m_operator, Constants.ButtonPorts.RT).onTrue(new SetArmAngleToStartPos());
    Trigger upperButton = new JoystickButton(m_operator, Constants.ButtonPorts.RB).onTrue(new MoveArmByAngle(-128,160)); // big
    Trigger midButton = new JoystickButton(m_operator, Constants.ButtonPorts.RT).onTrue(new MoveArmByAngle(-125, -288+360)); //mid
    Trigger collectCone = new JoystickButton(m_operator, Constants.ButtonPorts.START).onTrue(new MoveArmByAngle(120, -55)); //isuf mahmadaf
    Trigger collectCube = new JoystickButton(m_operator, Constants.ButtonPorts.BACK).onTrue(new MoveArmByAngle(105.49851545607983, -61.848)); //isuf mahmadaf
    Trigger cubeButton = new JoystickButton(m_operator, Constants.ButtonPorts.X).onTrue(new GripCube());
    Trigger switchModes = new JoystickButton(m_operator, Constants.ButtonPorts.LT).onTrue(new InstantCommand(()-> Gripper.getInstance().switchModes()));
    Trigger zeroPos = new JoystickButton(m_operator, Constants.ButtonPorts.LB).onTrue(new SetArmAngleToStartPos());

    // Trigger rightButton = new JoystickButton(m_operator, Constants.ButtonPorts.).onTrue(new MoveArmByAngle(-125, -288+360)); //mid
    // Trigger downButton = new JoystickButton(m_operator, Constants.ButtonPorts.A).onTrue(new MoveArmByAngle(144, -45)); //isuf mahmadaf


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

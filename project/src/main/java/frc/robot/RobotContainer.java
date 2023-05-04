// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.General.Commands;
import frc.robot.commands.Gripper.CloseGripper;
import frc.robot.commands.Gripper.GripCube;
import frc.robot.commands.Gripper.OpenGripper;
import frc.robot.commands.Arm.ArmMoveAndStayAtAngle;
import frc.robot.commands.Chassis.Slow;
import frc.robot.commands.Chassis.TankDrive;
import frc.robot.commands.Chassis.Turbo;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  
  public static final Joystick m_leftStick = new Joystick(Constants.JoystickPorts.leftJoystick);
  public static final Joystick m_rightStick = new Joystick(Constants.JoystickPorts.rightJoystick);
  public static final Joystick m_operator = new Joystick(Constants.JoystickPorts.operator);
  
  public static TankDrive m_tankDriveCommand = new TankDrive(m_rightStick::getY, m_leftStick::getY);
  
  public static Trigger up = new POVButton(m_operator, 0);
  public static Trigger down = new POVButton(m_operator, 180);
  public static Trigger left= new POVButton(m_operator, 270);
  public static Trigger right = new POVButton(m_operator, 90);
  
  private static final double ARM1_ANGLE_JUMPS = 5;
  private static final double ARM2_ANGLE_JUMPS = 15;
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

    Trigger closeGripper = new JoystickButton(m_operator, Constants.ButtonPorts.Y).onTrue(new CloseGripper().withTimeout(20));
    Trigger openGripper = new JoystickButton(m_operator, Constants.ButtonPorts.B).onTrue(new OpenGripper().withTimeout(20));
    Trigger closeToCube = new JoystickButton(m_operator, Constants.ButtonPorts.X).onTrue(new GripCube().withTimeout(20));
   
    up.onTrue(new InstantCommand(() -> new ArmMoveAndStayAtAngle(  SmartDashboard.getNumber("arm1-setpoint", Math.round(Arm.getFirstAngle())), Math.abs(Arm.getSecondAngle()) + ARM2_ANGLE_JUMPS, 30, false).schedule()));
    down.onTrue(new InstantCommand(() -> new ArmMoveAndStayAtAngle(SmartDashboard.getNumber("arm1-setpoint", Math.round(Arm.getFirstAngle())), Math.abs(Arm.getSecondAngle()) - ARM2_ANGLE_JUMPS, 30, false).schedule()));
    left.onTrue(new InstantCommand(() -> new ArmMoveAndStayAtAngle( Arm.getFirstAngle()- ARM1_ANGLE_JUMPS,  SmartDashboard.getNumber("arm2-setpoint", Math.abs(Math.round(Arm.getSecondAngle()/6)) * 6), 30, false).schedule()));
    right.onTrue(new InstantCommand(() -> new ArmMoveAndStayAtAngle(Arm.getFirstAngle() + ARM1_ANGLE_JUMPS, SmartDashboard.getNumber("arm2-setpoint", Math.abs(Math.round(Arm.getSecondAngle()/6)) * 6), 30, false).schedule()));

    up.onTrue(new InstantCommand(() -> new ArmMoveAndStayAtAngle(  SmartDashboard.getNumber("arm1-setpoint", Math.round(Arm.getFirstAngle())), SmartDashboard.getNumber("arm2-setpoint", Math.abs(Arm.getSecondAngle())) + ARM2_ANGLE_JUMPS, 30, false).schedule()));
    down.onTrue(new InstantCommand(() -> new ArmMoveAndStayAtAngle(SmartDashboard.getNumber("arm1-setpoint", Math.round(Arm.getFirstAngle())), SmartDashboard.getNumber("arm2-setpoint", Math.abs(Arm.getSecondAngle())) - ARM2_ANGLE_JUMPS, 30, false).schedule()));
    left.onTrue(new InstantCommand(() -> new ArmMoveAndStayAtAngle(SmartDashboard.getNumber("arm1-setpoint", Arm.getFirstAngle()) - ARM1_ANGLE_JUMPS,  SmartDashboard.getNumber("arm2-setpoint", Math.abs(Math.round(Arm.getSecondAngle()/6)) * 6), 30, false).schedule()));
    right.onTrue(new InstantCommand(() -> new ArmMoveAndStayAtAngle(SmartDashboard.getNumber("arm1-setpoint", Arm.getFirstAngle()) + ARM1_ANGLE_JUMPS, SmartDashboard.getNumber("arm2-setpoint", Math.abs(Math.round(Arm.getSecondAngle()/6)) * 6), 30, false).schedule()));
 
    Trigger upperCone = new JoystickButton(m_operator, Constants.ButtonPorts.RB).onTrue(new ArmMoveAndStayAtAngle  (-124, 120,Constants.ArmValues.PICKUP_TOLERANCE, false));
    Trigger lowerBackward = new JoystickButton(m_operator, Constants.ButtonPorts.RT).onTrue(new ArmMoveAndStayAtAngle  (-105, 83,Constants.ArmValues.PICKUP_TOLERANCE, false));
    Trigger setArmToZero = new JoystickButton(m_operator, Constants.ButtonPorts.START).onTrue(new ArmMoveAndStayAtAngle(0, 0,Constants.ArmValues.PICKUP_TOLERANCE, false)); 

    Trigger lowerCone = new JoystickButton(m_operator, Constants.ButtonPorts.LT).onTrue(new ArmMoveAndStayAtAngle(-124, 70, Constants.ArmValues.PICKUP_TOLERANCE, false));
    Trigger pickUpCube = new JoystickButton(m_operator, Constants.ButtonPorts.LB).onTrue(new ArmMoveAndStayAtAngle(115, 70, 5, false));
    Trigger upperCube = new JoystickButton(m_operator, Constants.ButtonPorts.BACK).onTrue(new ArmMoveAndStayAtAngle(-120, 105, Constants.ArmValues.PICKUP_TOLERANCE, false));
    Trigger collectFromFloor = new JoystickButton(m_operator, Constants.ButtonPorts.A).onTrue(new ArmMoveAndStayAtAngle(-10, -110, Constants.ArmValues.PICKUP_TOLERANCE, false));
 
    Trigger turbo = new JoystickButton(m_rightStick, 1).whileTrue(new Turbo(m_tankDriveCommand));
    Trigger slow = new JoystickButton(m_rightStick, 2).whileTrue(new Slow(m_tankDriveCommand));
    Trigger lockWheels = new JoystickButton(m_rightStick, 3).onTrue(Commands.lockWheels);
    
  }

}

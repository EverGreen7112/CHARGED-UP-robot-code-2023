// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

public class GoToReflectorSmallBrain extends CommandBase {
  /** Creates a new GoToReflectorSmallBrain. */
  PIDController pid = new PIDController(0.05 / 44, 0.00006, 0.00004);
  double driveSpeed = -0.0;
  public GoToReflectorSmallBrain() {
    addRequirements(Chassis.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleSpeed = pid.calculate(Chassis.getAngleToReflector(), 0);
    Chassis.driveTank(driveSpeed + angleSpeed, driveSpeed - angleSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_tankDriveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return RobotContainer.m_leftStick.getRawButtonReleased(4);
  }
}

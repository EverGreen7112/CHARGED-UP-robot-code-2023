// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class CloseGripperOnZero extends CommandBase {
  /** Creates a new CloseGripperOnZero. */
  public CloseGripperOnZero() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Gripper.getInstance());
  }
  CloseGripper close = new CloseGripper();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!Gripper.getCube().get() && !Gripper.getCube().get() ) { // more safety than checking opened switch
      close.schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return close.isFinished();
  }
}

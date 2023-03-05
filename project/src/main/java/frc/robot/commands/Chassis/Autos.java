// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    //return new AutoMove(1.6, 0.3);
   // return new SequentialCommandGroup(new MoveArmByAngle(128,160),new AutoMove(1.6, 0.3), new OpenGripper(), new CloseGripper(),new AutoMove(1.6, -0.3));
    //return new SequentialCommandGroup(new MoveArmByAngle(-128,160), new OpenGripper(), new CloseGripper());
    //return new SequentialCommandGroup( new TurnArmTwo(-25), new OpenGripper(), new CloseGripper(), new AutoMove(1.6, 0.3));
    return null;
  }


  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class StallBoth extends CommandBase{
    @Override
    public void initialize() {
       addRequirements(Arm.getInstance());
    }
    @Override
    public void execute() {
      Arm.stall2();
      Arm.stall1();
   }
}

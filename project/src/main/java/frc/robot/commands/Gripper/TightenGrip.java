package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class TightenGrip extends CommandBase{
    public TightenGrip(){
        addRequirements(Gripper.getInstance());
    }
    @Override
    public void execute() {
        Gripper.moveGripper(-0.1);
    }
    @Override
    public void end(boolean interrupted) {
        Gripper.stop();
    }
    
}

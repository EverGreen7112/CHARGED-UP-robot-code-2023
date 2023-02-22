package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class CloseGripper extends CommandBase{

    private Gripper gripper;

    public CloseGripper() {
        addRequirements(Gripper.getInstance());
    }

    @Override
    public void initialize() {
        gripper = Gripper.getInstance();
    }

    @Override
    public void execute() {
        gripper.closeGripper();
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
    
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class OpenGripper extends CommandBase {

    private Gripper gripper;

    public OpenGripper() {
        addRequirements(Gripper.getInstance());
    }

    @Override
    public void initialize() {
        gripper = Gripper.getInstance();
    }

    @Override
    public void execute() {
        gripper.openGripper();
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
    
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gripper;

public class GripCube extends CommandBase {
    
    private Gripper gripper;
    private DigitalInput limitSwitch;

    public GripCube() {
        addRequirements(Gripper.getInstance());
    }

    @Override
    public void initialize() {
        gripper = Gripper.getInstance();
        //not sure if this port is the correct one.
        limitSwitch = new DigitalInput(Constants.Ports.CUBE_LIMIT_SWITCH);
    }

    @Override
    public void execute() {
        gripper.closeGripper();
    }

    @Override
    public boolean isFinished() {
        return limitSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
}

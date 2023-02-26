package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gripper;

public class OpenGripper extends CommandBase {

    private Gripper gripper;
    private DigitalInput limitSwitch;

    public OpenGripper() {
        addRequirements(Gripper.getInstance());
    }

    @Override
    public void initialize() {
        gripper = Gripper.getInstance();
        //not sure if this port is the correct one.
        limitSwitch = new DigitalInput(Constants.Ports.FIRST_LIMIT_SWITCH);
    }

    @Override
    public void execute() {
        gripper.openGripper();
    }

    @Override
    public boolean isFinished() {
        //if limit switch is pressed stops the motor
        return limitSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
    
}

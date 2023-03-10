package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gripper;

public class OpenGripper extends CommandBase {

    private DigitalInput limitSwitch;

    public OpenGripper() {
        addRequirements(Gripper.getInstance());
        limitSwitch = Gripper.getOpened();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Gripper.openGripper();
    }

    @Override
    public boolean isFinished() {
        //if limit switch is pressed stops the motor
        return !limitSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        Gripper.stop();
    }
    
}

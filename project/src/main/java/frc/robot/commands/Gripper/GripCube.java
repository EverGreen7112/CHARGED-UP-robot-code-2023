package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gripper;

public class GripCube extends CommandBase {
    
    private Gripper gripper;
    private DigitalInput limitSwitch;

    public GripCube() {
        addRequirements(Gripper.getInstance());
        limitSwitch = Gripper.getInstance().getCube();
    }

    @Override
    public void initialize() {
        gripper = Gripper.getInstance();
    }

    @Override
    public void execute() {
        gripper.closeGripperCube();
    }

    @Override
    public boolean isFinished() {
        return !limitSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
}

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PidOldValuesDontUse;
import frc.robot.subsystems.Arm;

public class MoveArm1ByAngleAndJoystick extends CommandBase{
    private PIDController cont;
    public MoveArm1ByAngleAndJoystick(double desierdAngle) {
        addRequirements(Arm.getInstance());
        cont.setP(PidOldValuesDontUse.FIRST_ARM_ANTI_KP);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }
}

package frc.robot.commands.Arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class JoystickArmControll extends CommandBase {
    private double range = 0.4;
    private double m_lastOutput;

    
    public JoystickArmControll() {
        addRequirements(Arm.getInstance());
    }
    @Override
    public void initialize() {
        m_lastOutput = Arm.getFirst().get();
    }
    @Override
    public void execute() {
        Arm.getFirst().set(RobotContainer.m_operator.getY() * range + m_lastOutput);
        Arm.getSecond().set(RobotContainer.m_operator.getZ() * range + m_lastOutput);
    }

}

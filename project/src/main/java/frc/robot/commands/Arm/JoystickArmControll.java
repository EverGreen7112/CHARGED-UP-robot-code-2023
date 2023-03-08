package frc.robot.commands.Arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class JoystickArmControll extends CommandBase {
    private double range = 0.4;
    private Supplier<Double> m_lastOutput;

    
    public JoystickArmControll(Supplier<Double> lastPow) {
        addRequirements(Arm.getInstance());
        m_lastOutput = lastPow;
    }

    @Override
    public void execute() {
        Arm.getFirst().set(RobotContainer.m_operator.getY() * range + m_lastOutput.get());
        Arm.getSecond().set(RobotContainer.m_operator.getZ() * range + m_lastOutput.get());
    }

}

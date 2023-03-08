package frc.robot.commands.Arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class StallArm1 extends CommandBase{
    public StallArm1() {
        addRequirements(Arm.getInstance());
    }
    @Override
    public void execute() {
        Arm.getFirst().set(TalonFXControlMode.PercentOutput,Arm.getFirstStall());
    }
}
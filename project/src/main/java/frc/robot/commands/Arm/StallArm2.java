package frc.robot.commands.Arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class StallArm2 extends CommandBase{
    @Override
    public void initialize() {
       addRequirements(Arm.getInstance());
    }
    @Override
    public void execute() {
      Arm.getSecond().set(Arm.getSecondStall());
      
   }
}

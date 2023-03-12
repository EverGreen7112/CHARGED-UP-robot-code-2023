package frc.robot.commands.Chassis;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class LockWheels extends CommandBase{
   
    @Override
    public void initialize() {
        if(Chassis.getMode() == IdleMode.kBrake){
            Chassis.setMode(IdleMode.kCoast);
        }
        else {
            Chassis.setMode(IdleMode.kBrake);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
   
    
}

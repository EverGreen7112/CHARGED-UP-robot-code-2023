package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vector2D;
import frc.robot.subsystems.Chassis;

public class TankDrive extends CommandBase{
    
    private Supplier<Double> m_rightSupllier;
    private Supplier<Double> m_leftSupllier;

    public TankDrive(Supplier<Double> left, Supplier<Double> right){
        addRequirements(Chassis.getInstance());
        m_rightSupllier=right;
        m_leftSupllier=left;
    }
    
    @Override
    public void execute() {
        if (Math.abs(m_leftSupllier.get()) < 0.3 && Math.abs(m_rightSupllier.get()) < 0.3) {
            return;
        }
        double rSpeed=-m_rightSupllier.get();
        double lSpeed=-m_leftSupllier.get();
        Vector2D v=new Vector2D(lSpeed, rSpeed);
        if(v.getLength()>Constants.Speeds.driveMax.get()){
            //normalizing the vector.
            v.x/=(v.getLength());
            v.y/=(v.getLength());
            v.x*=Constants.Speeds.driveMax.get();
            v.y*=Constants.Speeds.driveMax.get();
        }
        lSpeed=v.x;
        rSpeed=v.y;
        Chassis.getInstance().driveTank(lSpeed * Constants.Speeds.constantSpeed, rSpeed * Constants.Speeds.constantSpeed);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Chassis.getInstance().stop();
    }
    
}

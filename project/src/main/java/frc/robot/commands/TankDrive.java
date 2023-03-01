package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vector2D;
import frc.robot.subsystems.Chassis;

public class TankDrive extends CommandBase{
    
    private Supplier<Double> m_rightSupllier;
    private Supplier<Double> m_leftSupllier;

    private boolean m_turbo;
    private boolean m_slow;

    public TankDrive(Supplier<Double> left, Supplier<Double> right){
        addRequirements(Chassis.getInstance());
        m_rightSupllier=right;
        m_leftSupllier=left;
    }
    
    @Override
    public void execute() {
        if (Math.abs(m_leftSupllier.get()) < 0.2 && Math.abs(m_rightSupllier.get()) < 0.2) {
            Chassis.getInstance().stop();
            return;
        }
        double rSpeed=-m_rightSupllier.get();
        double lSpeed=-m_leftSupllier.get();
        Vector2D v=new Vector2D(lSpeed, rSpeed);
        if(v.getLength()>Constants.Speeds.constantSpeed.get()){
            //normalizing the vector.
            v.x/=(v.getLength());
            v.y/=(v.getLength());
            v.x*=Constants.Speeds.constantSpeed.get();
            v.y*=Constants.Speeds.constantSpeed.get();
        }
        lSpeed=v.x * ((m_turbo) ? Constants.Speeds.TURBO : (m_slow) ? Constants.Speeds.SLOW : 1);
        rSpeed=v.y * ((m_turbo) ? Constants.Speeds.TURBO : (m_slow) ? Constants.Speeds.SLOW : 1);
        Chassis.getInstance().driveTank(lSpeed * Constants.Speeds.constantSpeed.get(), rSpeed * Constants.Speeds.constantSpeed.get());
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Chassis.getInstance().stop();
    }
    public void setTurbo(boolean turbo){
        m_turbo = turbo;
    }
    public void setSlow(boolean slow){
        m_slow = slow;
    }
    
    
}

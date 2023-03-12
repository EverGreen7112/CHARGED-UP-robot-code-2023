package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Slow extends CommandBase{
    private TankDrive m_instance;
    
    public Slow(TankDrive instance) {
        m_instance = instance;
    }
    @Override
    public void initialize(){
        m_instance.setSlow(true);
    }
    @Override
    public void execute() {
        m_instance.setSlow(true);
    }
    @Override
    public void end(boolean a){
        m_instance.setSlow(false);
    }
    
}

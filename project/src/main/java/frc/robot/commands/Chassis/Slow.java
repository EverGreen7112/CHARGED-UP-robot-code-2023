package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Slow extends CommandBase{
    private TankDrive m_instance;
    
    public Slow(TankDrive instance) {
        m_instance = instance;
    }
    @Override
    public void initialize(){
        m_instance.setTurbo(false);
        m_instance.setSlow(true);
    }
    @Override
    public void end(boolean a){
        m_instance.setTurbo(false);
        m_instance.setSlow(false);
    }
    
}

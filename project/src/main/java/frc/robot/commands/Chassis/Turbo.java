package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Turbo extends CommandBase{
    private TankDrive m_instance;
    
    public Turbo(TankDrive instance) {
        m_instance = instance;
    }
    @Override
    public void initialize(){
        m_instance.setTurbo(true);
    }
    @Override
    public void execute() {
        m_instance.setTurbo(true);
    }
    @Override
    public void end(boolean a){
        m_instance.setTurbo(false);
    }
    

}

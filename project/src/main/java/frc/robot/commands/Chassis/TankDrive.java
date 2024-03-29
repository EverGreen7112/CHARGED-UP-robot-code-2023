package frc.robot.commands.Chassis;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        if (Math.abs(m_leftSupllier.get()) < 0.3 && Math.abs(m_rightSupllier.get()) < 0.3) {
            Chassis.stop();
            return;
        }
        double rSpeed=-m_rightSupllier.get();
        double lSpeed=-m_leftSupllier.get();
        Vector2D v=new Vector2D(lSpeed, rSpeed);

         double turboSpeed = SmartDashboard.getNumber("Turbo", Constants.Speeds.TURBO);
         double slowSpeed  = SmartDashboard.getNumber("slow", Constants.Speeds.SLOW);
         double normalSpeed = Constants.Speeds.constantSpeed.get();
        // lSpeed=v.x * ((m_turbo) ? turboFactor : ((m_slow) ? slowFactor : 1));
        // rSpeed=v.y * ((m_turbo) ? turboFactor : ((m_slow) ? slowFactor : 1));

        double cosntantSpeed = ((m_turbo) ? turboSpeed : ((m_slow) ? slowSpeed : normalSpeed));
        // SmartDashboard.putNumberArray("l, r", new double[] {lSpeed, rSpeed});
        Chassis.driveTank(lSpeed * cosntantSpeed, rSpeed * cosntantSpeed);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Chassis.stop();
    }
    public void setTurbo(boolean turbo){
        m_turbo = turbo;
    }
    public void setSlow(boolean slow){
        m_slow = slow;
    }
    
    
}

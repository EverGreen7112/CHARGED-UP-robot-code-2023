package frc.robot.commands.Chassis;

import java.lang.annotation.Target;
import java.lang.constant.Constable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class AutoMove extends CommandBase {

    private double m_timeToMove;
    private double m_time;
    private double m_speed;
    
    public AutoMove(double seconds, double speed) {
       m_timeToMove = seconds * 1000;
       m_speed = speed;

    }

    @Override
    public void initialize() {
        m_time = System.currentTimeMillis();
        Chassis.getInstance().driveStraight(m_speed);   

    }
    @Override
    public void end(boolean interrupted) {
      Chassis.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - m_time) >= m_timeToMove;
        //return false;
        // return m_currentDistance > m_targetMeters; 
    }
}

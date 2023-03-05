package frc.robot.commands;

import javax.swing.text.AbstractDocument.BranchElement;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

public class AutoRouteOne extends CommandBase {

    double m_startTime;

    @Override
    public void initialize() {
        //startDistance = Constants.Conversions.ticksToMeters(Chassis.getInstance().getEncodersDist(), );
        m_startTime = System.currentTimeMillis();
        RobotContainer.upperSmall.schedule();
        // Thread.sleep(3000);
        new WaitCommand(3.0).schedule();
        new OpenGripper().schedule();
    }

    @Override
    public void execute() {
        if(System.currentTimeMillis() > m_startTime + 3000 && System.currentTimeMillis() < m_startTime + 7000) {
            Chassis.getInstance().driveStraight(-0.3);
        }
        else {
            Chassis.getInstance().driveStraight(0);
        }         
    }

    @Override
    public void end(boolean interrupted) {
       // Chassis.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
    //    return (System.currentTimeMillis() - m_startTime) >= m_timeToMove;
        return false;
    }
}

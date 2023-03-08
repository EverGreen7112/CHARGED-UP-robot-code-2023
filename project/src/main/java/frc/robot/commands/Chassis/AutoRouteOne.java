package frc.robot.commands.Chassis;

import javax.swing.text.AbstractDocument.BranchElement;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.General.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Gripper.OpenGripper;
import frc.robot.subsystems.Chassis;

public class AutoRouteOne extends CommandBase {

    double m_startTime;

    @Override
    public void initialize() {
        //startDistance = Constants.Conversions.ticksToMeters(Chassis.getEncodersDist(), );
        m_startTime = System.currentTimeMillis();
        Commands.upperSmall.schedule();
        // Thread.sleep(3000);
        new WaitCommand(3.0).schedule();
        new OpenGripper().schedule();
    }

    @Override
    public void execute() {
        if(System.currentTimeMillis() > m_startTime + 3000 && System.currentTimeMillis() < m_startTime + 7000) {
            Chassis.driveStraight(-0.3);
        }
        else {
            Chassis.driveStraight(0);
        }         
    }

    @Override
    public void end(boolean interrupted) {
       // Chassis.stop();
    }

    @Override
    public boolean isFinished() {
    //    return (System.currentTimeMillis() - m_startTime) >= m_timeToMove;
        return false;
    }
}

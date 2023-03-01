package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class AutoRouteOne extends CommandBase {
    //private double startDistance;
    private long m_startTime;
    private long m_timeToMove;

    /**
     * @param timeToMove time to move robot in millis
     */
    public AutoMoveBack(long timeToMove) {
        m_timeToMove = timeToMove;
    }

    @Override
    public void initialize() {
        //startDistance = Constants.Conversions.ticksToMeters(Chassis.getInstance().getEncodersDist(), );
        m_startTime = System.currentTimeMillis();
        Chassis.getInstance().driveStraight(0.4);
    }

    @Override
    public void execute() {
        Chassis.getInstance().driveStraight(0.4);   
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

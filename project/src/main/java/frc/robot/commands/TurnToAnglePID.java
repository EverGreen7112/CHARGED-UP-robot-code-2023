package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class TurnToAnglePID extends CommandBase{
    private double m_targetAngle;
    private double kp = 0.0025;

    public TurnToAnglePID(double angle){
        m_targetAngle = angle;
    }


    @Override
    public void execute() {
        Chassis.getInstance().driveTank(kp * (m_targetAngle - Chassis.getGyro().getAngle()),-1 * kp * (m_targetAngle - Chassis.getGyro().getAngle()) );
        SmartDashboard.putNumber("one",  kp * Chassis.getGyro().getAngle());
        SmartDashboard.putNumber("two", -1 * kp * Chassis.getGyro().getAngle());
    }

    @Override
    public boolean isFinished() {
        return Chassis.getGyro().getAngle() > m_targetAngle - 3 && Chassis.getGyro().getAngle() < m_targetAngle + 3;  
    }
    @Override
    public void end(boolean interrupted) {
        Chassis.getInstance().stop();
    }
}

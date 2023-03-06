package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToAnglePID extends CommandBase{
    private double m_targetAngle;
    public TurnToAnglePID(double angle){
        m_targetAngle = angle;
    }
}

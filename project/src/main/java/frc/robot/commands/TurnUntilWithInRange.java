package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

/**
 * Turn until robot within range of angles, if already in range will not move
 */
public class TurnUntilWithInRange extends CommandBase {
    public double m_minAng, m_maxAng;
    private double m_endAng = -1;
    @Override
    public void schedule(boolean interruptible) {
        // TODO Auto-generated method stub
        super.schedule(interruptible);
    }

    public TurnUntilWithInRange(double minAng, double maxAng) {
        addRequirements(Chassis.getInstance());
        m_minAng = minAng;
        m_maxAng = maxAng;
    }

    private double calcAng(double a) {
        return (a % 360 + 360) % 360;
    }

    private boolean inRange() {
        return !(m_minAng > Chassis.getInstance().getRobotAngle() || m_maxAng < Chassis.getInstance().getRobotAngle());
    }

    @Override
    public void initialize() {
        double curAngle = Chassis.getInstance().getRobotAngle();
        if (!inRange()) {
            double rTurnAngDist = calcAng(curAngle - m_maxAng);
            double lTurnAngDist = calcAng(curAngle - m_minAng);
            double finalSpeed = Constants.Speeds.constantSpeed
                    + Constants.Speeds.constantSpeed * Math.min(rTurnAngDist, lTurnAngDist);
            if (rTurnAngDist < lTurnAngDist) {
                Chassis.getInstance().turnRight(finalSpeed);
            } else {
                Chassis.getInstance().turnLeft(finalSpeed);
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (inRange()&&this.isScheduled()) {
            m_endAng = Chassis.getInstance().getRobotAngle();
        }
        return inRange();
    }

    @Override
    public void end(boolean interrupted) {
        Chassis.getInstance().stop();
    }

    /**
     * 
     * @return the angle in which the command has been finished, if the command
     *         havent been finished return -1,
     *         <p>
     *         edge case: if the command ran more then one time it will return the
     *         angle at the last time the robot stopped
     *         </p>
     */
    public double endAng() {
        return m_endAng;
    }

}

package frc.robot.commands.General;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Vector2D;
import frc.robot.commands.Arm.TurnUntilWithInRange;
import frc.robot.commands.ChassisPid.ChasisSetPointPosPID;
import frc.robot.subsystems.Chassis;

public class Commands {
    /**
     * <p>
     * Commands that go from one point to the other using simple drive and pid
     * sequntally, assuming that turns at the endpoint are not desirebale
     * </p>
     * Symbols:
     * <ul>
     * <li>l1 stright line at the angle of the robot that goes through the
     * robot</li>
     * <li>l2: stright line at the angle of the endpoint that goes through the
     * endpoint</li>
     * <li>R will represent the robot position, and E the endpoint</li>
     * </ul>
     * 
     * Steps:
     * <ol>
     * <li>(if necessery) The robot will turn until the intresection between l1 and
     * l2 is within miLineDist and maxLineDist(in the opsite direction of the bot)
     * (this will not be done with pid, but rather turn and when in range stop, for
     * time saving (the speed will be b+m*(diffrence_between_angles)))</li>
     * <li>The robot will drive stright to this intresection point</li>
     * <li>the robot will turn until l1 will overlap with l2</li>
     * <li>the robot will turn until p overlap with E</li>
     * </ol>
     * 
     * @param curX        current x cordinite of robot
     * @param curY        current y cordinite of robot
     * @param curAng      current angle of robot
     * @param endX        desired x cordinite of robot
     * @param endY        desired y cordinite of robot
     * @param endAng      desired angle of robot
     * @param minLineDist the minimum line distance (see first step)
     * @param maxLineDist the maximum line distance (see first step)
     * @return the command
     */
    public static CommandBase getDriveToLocationCommand(Vector2D startP, double curAng, Vector2D endP, double endAng,
            double minLineDist, double maxLineDist) {
        // TODO: maybe can be optimized using reverse
        // TODO:!!! replace pid with canspark pid
        // calculate desired range of angles that will cause stright line from the robot
        // to be in the range of minLineDist to maxLineDist
        Vector2D endDir = new Vector2D(Math.cos(endAng), Math.sin(endAng));
        Vector2D minP = endP.getSubtracted(endDir.getMultiplied(minLineDist));
        Vector2D maxP = endP.getSubtracted(endDir.getMultiplied(maxLineDist));
        Vector2D selfToMin = minP.getSubtracted(startP);
        Vector2D selfToMax = maxP.getSubtracted(startP);
        Vector2D e1 = new Vector2D(1, 0);
        double minAng = selfToMin.getAngle();
        double maxAng = selfToMax.getAngle();
        double minimumAngle = Math.min(minAng, maxAng);// while minAng is the angle of the vector to the minLine point
                                                       // minimumAng is the minimum angle between minAng and maxAng
        double maximumAngle = Math.max(maxAng, minAng);
        TurnUntilWithInRange turnUntilWithinRange = new TurnUntilWithInRange(minimumAngle, maximumAngle);
        // Calculate desired distance
        DoubleSupplier intresectingSetPoint = () -> {
            double cAng = turnUntilWithinRange.endAng();
            double m1 = Math.tan(cAng);
            double m2 = Math.tan(endAng);
            double b2 = endP.y - startP.y - (endP.x - startP.x) * m2;
            double xIntresection = b2 / (m1 - m2);
            double yIntresection = m1 * xIntresection;
            Vector2D intersection = new Vector2D(xIntresection, yIntresection);
            return intersection.getLength();
        };
        // create pid command to drive to desired distance
        PIDController driveController = new PIDController(Constants.PIDS.driveKp, Constants.PIDS.driveKi,
                Constants.PIDS.driveKd);
        double startLocation = Chassis.getInstance().getEncodersDist();
        driveController.setTolerance(Constants.PIDS.drivePTolerance, Constants.PIDS.driveVTolerance);
        CommandBase driveToIntresection = new ChasisSetPointPosPID(intresectingSetPoint);

        // create pid command to rotate to desired angle
        PIDController rotateController = new PIDController(Constants.PIDS.rotateKp, Constants.PIDS.rotateKi,
                Constants.PIDS.rotateKd);
        rotateController.setTolerance(Constants.PIDS.rotatePTolerance, Constants.PIDS.rotateVTolerance);
        rotateController.enableContinuousInput(0, 360);
        PIDCommand rotateToOverlap = new PIDSetPointCommand(
                rotateController,
                () -> Chassis.getInstance().getRobotAngle(), endAng,
                Chassis.getInstance()::turnLeft, (Subsystem) Chassis.getInstance());// might be turn right
        //create pid command to drive for p and e to overlap
        DoubleSupplier overlapSetPoint = () -> {
            double cAng = turnUntilWithinRange.endAng();
            double m1 = Math.tan(cAng);
            double m2 = Math.tan(endAng);
            double b2 = endP.y - startP.y - (endP.x - startP.x) * m2;
            double xIntresection = b2 / (m1 - m2);
            double yIntresection = m1 * xIntresection;
            Vector2D intersection = new Vector2D(xIntresection, yIntresection);
            Vector2D normalEnd = endP.getSubtracted(startP);
            Vector2D intreToEnd = normalEnd.getSubtracted(intersection);
            return intreToEnd.getLength();
        };
        CommandBase driveToOverlap = new ChasisSetPointPosPID(overlapSetPoint);
        
        return new SequentialCommandGroup(turnUntilWithinRange,driveToIntresection,rotateToOverlap,driveToOverlap);
    }

    

}
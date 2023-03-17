package frc.robot.commands.General;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Vector2D;
import frc.robot.Constants.ArmValues;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveDistanceByEncoders;
import frc.robot.commands.DriveUntilIsTilted;
import frc.robot.commands.Arm.ArmMoveAndStayAtAngle;
import frc.robot.commands.Arm.JoystickArmControll;
import frc.robot.commands.Arm.MoveArm1ByAngle;
import frc.robot.commands.Arm.MoveArm2ByAngle;
import frc.robot.commands.Arm.StallArm1;
import frc.robot.commands.Arm.StallBoth;
import frc.robot.commands.Chassis.LockWheels;
import frc.robot.commands.ChassisPid.ChasisSetPointPosPID;
import frc.robot.commands.Gripper.GripCube;
import frc.robot.commands.Gripper.OpenGripper;
import frc.robot.commands.Gripper.TightenGrip;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gripper;

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
    /*  public static CommandBase getDriveToLocationCommand(Vector2D startP, double curAng, Vector2D endP, double endAng,
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
        // turnUntilWithinRange = new TurnUntilWithInRange(minimumAngle, maximumAngle);
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
        double startLocation = Chassis.getEncodersDist();
        driveController.setTolerance(Constants.PIDS.drivePTolerance, Constants.PIDS.driveVTolerance);
        CommandBase driveToIntresection = new ChasisSetPointPosPID(intresectingSetPoint);

        // create pid command to rotate to desired angle
        PIDController rotateController = new PIDController(Constants.PIDS.rotateKp, Constants.PIDS.rotateKi,
                Constants.PIDS.rotateKd);
        rotateController.setTolerance(Constants.PIDS.rotatePTolerance, Constants.PIDS.rotateVTolerance);
        rotateController.enableContinuousInput(0, 360);
        PIDCommand rotateToOverlap = new PIDSetPointCommand(
                rotateController,
                () -> Chassis.getRobotAngle(), endAng,
                Chassis::turnLeft, (Subsystem) Chassis.getInstance());// might be turn right
        // create pid command to drive for p and e to overlap
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

        return new SequentialCommandGroup(turnUntilWithinRange, driveToIntresection, rotateToOverlap, driveToOverlap);
    }
*/
    public static CommandBase toggleConeIn = new InstantCommand(() -> Arm.toggleConeIn());
    public static CommandBase lockWheels = new LockWheels();
    public static CommandBase invertChassis = new InstantCommand(() -> {
        Chassis.m_rightFrontEngine.setInverted(!Chassis.getInstance().m_rightFrontEngine.getInverted());
        Chassis.m_leftFrontEngine.setInverted(!Chassis.getInstance().m_leftFrontEngine.getInverted());
        // SmartDashboard.putBoolean("AaaaAAA", Chassis.getInstance().m_rightFrontEngine.getInverted());
    });
    public static CommandBase upperBig = new RunCommand(
            () -> Arm.getFirst().set( 0.3), Arm.getInstance()) {
        @Override
        public void end(boolean interrupted) {
            Arm.getFirst().set( 0);
        }
    };
    public static CommandBase lowerBig = new RunCommand(
            () -> Arm.getFirst().set( -0.3), Arm.getInstance()) {
        @Override
        public void end(boolean interrupted) {
            Arm.getFirst().set( 0);
        }
    };
    public static CommandBase upperSmall = new RunCommand(
            () -> Arm.getSecond().set( 0.12 + (RobotContainer.m_operator.getY() * 0.5)),
            Arm.getInstance()) {
        @Override
        public void end(boolean interrupted) {
            Arm.getSecond().set( 0);
        }
    };
    public static CommandBase lowerSmall = new RunCommand(
            () -> Arm.getSecond().set( -0.12 + (RobotContainer.m_operator.getY() * 0.5)),
            Arm.getInstance()) {
        @Override
        public void end(boolean interrupted) {
            Arm.getSecond().set( 0);
        }
    };
    // public static CommandBase tightenGrip = new RunCommand(() -> Gripper.moveGripper(-0.1), Gripper.getInstance());
    public static CommandBase tightenGrip = new TightenGrip();
 
    public static CommandBase getMoveArm1ToAng(double desierdAng){
        return getMoveArm1ToAng(()->desierdAng);

    }
    public static CommandBase getMoveArm1ToAng(Supplier<Double> desierdAng){
        return new MoveArm1ByAngle(desierdAng).andThen(new StallArm1());

    }
    public static CommandBase getJoysticControl(){
        return new JoystickArmControll();
    }

    public static Command placeGamePieceAuto =  new SequentialCommandGroup(
      new ArmMoveAndStayAtAngle(0, 0, Constants.ArmValues.PICKUP_TOLERANCE, true) ,
      new ParallelRaceGroup(new ArmMoveAndStayAtAngle(-115, 120,Constants.ArmValues.PICKUP_TOLERANCE, true), new TightenGrip()),
      new ParallelRaceGroup(new ArmMoveAndStayAtAngle(-115, 120,Constants.ArmValues.PICKUP_TOLERANCE, false), new DriveDistanceByEncoders(-0.3, 0.05, 0.2)),
      new ParallelRaceGroup(new ArmMoveAndStayAtAngle(-115, 120,Constants.ArmValues.PICKUP_TOLERANCE, false),new OpenGripper().withTimeout(0.5)),
      new ParallelRaceGroup(new ArmMoveAndStayAtAngle(-115, 120,Constants.ArmValues.PICKUP_TOLERANCE, false),new GripCube()),
      new ParallelRaceGroup(new ArmMoveAndStayAtAngle(-115, 120,Constants.ArmValues.PICKUP_TOLERANCE, false),new DriveDistanceByEncoders(3.2, 0.01, 0.05)),
      new ArmMoveAndStayAtAngle(0, 0, Constants.ArmValues.PICKUP_TOLERANCE, true));

    public static Command placeGamePieceAndBalanceAuto =  new SequentialCommandGroup(
      new ArmMoveAndStayAtAngle(-115, 120,Constants.ArmValues.PICKUP_TOLERANCE, true),
      new ParallelRaceGroup(new ArmMoveAndStayAtAngle(-115, 120,Constants.ArmValues.PICKUP_TOLERANCE, false), new DriveDistanceByEncoders(-0.3, 0.05, 0.1)),
      new ParallelRaceGroup(new ArmMoveAndStayAtAngle(-115, 120,Constants.ArmValues.PICKUP_TOLERANCE, false), new OpenGripper().withTimeout(0.8)),
      new DriveDistanceByEncoders(0.5, 0.05, 0.4),
      new ParallelDeadlineGroup(new DriveUntilIsTilted(), new ArmMoveAndStayAtAngle(0, 0, 0, false), new GripCube()),
      new ParallelCommandGroup(new Balance(), new ArmMoveAndStayAtAngle(0, 0, 0, false)));
  
  
      public static Command balanceOnlyAuto =
   new SequentialCommandGroup( new ParallelRaceGroup(new DriveUntilIsTilted(), new ArmMoveAndStayAtAngle(0, 0, 0, false), new GripCube()),
   new ParallelRaceGroup(new Balance(), new ArmMoveAndStayAtAngle(0, 0, 0, false)));

   public static Command driveOnlyAuto = new ParallelRaceGroup(new DriveDistanceByEncoders(3, 0.05 , 0.05), new ArmMoveAndStayAtAngle(0, 0, 0, false));
    
    

   
    
}

package frc.robot.commands.unused;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vector2D;
import frc.robot.subsystems.Arm;

public class MoveArmByPos extends CommandBase {

    private Vector2D destination; // where the arm needs to go
    private Vector2D startPos = new Vector2D(0, 0); // where the hand begins
    private double arm1Length = Constants.ArmValues.FIRST_ARM_LENGTH;// length of the arm 1
    private double arm2Length = Constants.ArmValues.SECOND_ARM_LENGTH;// length of the arm 2
    private double height; // height of the vector between startPos and destination
    private double width; // width of the vector between startPos and destination
    private double arm1Angle; // the angle the first arm in degrees
    private double arm2Angle; // the angle the second arm in degrees
    private double pointsVecLength; // the length of the vector of the points

    public MoveArmByPos(double x, double y){
        destination = new Vector2D(x, y);
    }

    @Override
    public void initialize() {
        CalcAngles();
    }

    @Override
    public void execute(){
        CalcAngles();
        if (Possible()) {
            Arm.getInstance().turnFirstTo(arm1Angle);
            Arm.getInstance().turnSecondTo(arm2Angle);
        }
    }

    // calc and update the angles of the 2 arms
    private void CalcAngles() {
        height = Math.abs(startPos.y - destination.y);
        width = Math.abs(startPos.x - destination.x);

        pointsVecLength = Math.sqrt(height * height + width * width);

        arm1Angle = Math.toDegrees(Math.atan(height / width)) + CalcAngleLawOfCosines(arm2Length, arm1Length, pointsVecLength);
        arm2Angle = CalcAngleLawOfCosines(pointsVecLength,arm1Length,arm2Length);
    }

    // returns the angle var in the law of cosines (x) : a^2 = b^2 + c^2 - 2 * b * c
    // * cos(x)
    private double CalcAngleLawOfCosines(double a, double b, double c) {
        return Math.toDegrees(Math.acos((a * a - b * b - c * c) / (-2 * b * c)));
    }

    // is it possible to move the arm to destination
    private boolean Possible() {
        return Math.abs(arm1Length) + Math.abs(arm2Length) >= Math.abs(pointsVecLength);
    }
}

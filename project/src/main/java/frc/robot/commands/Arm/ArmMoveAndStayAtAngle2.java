// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;

//changes mark as CHANGES, beware, probably wont work too well on back to 0,0 since while changing i wasent considerd this case
public class ArmMoveAndStayAtAngle2 extends CommandBase {
    private double _kfArm1 = 0.053, _arm1Kp = 0.0054, _kfArm2 = 0.03, _kpArm2 = 0.0047, m_notHitAngle = 40;
    private double _arm1SetPoint, _arm2SetPoint, _tolerance;
    private boolean _allowEnd = true;

    /** Creates a new MoveAndStayAtAngle. */
    public ArmMoveAndStayAtAngle2(double desiredAngle, double tolerance) {
        setArm1Setpoint(desiredAngle);
        _tolerance = tolerance;
        addRequirements(Arm.getInstance());
    }

    public ArmMoveAndStayAtAngle2(double desiredAngleArm1, double desiredAngleArm2, double tolerance) {

        setArm1Setpoint(desiredAngleArm1);
        setArm2Setpoint(desiredAngleArm2);
        _tolerance = tolerance;
        addRequirements(Arm.getInstance());

    }

    public ArmMoveAndStayAtAngle2(double desiredAngle, double tolerance, boolean allowEnd) {
        setArm1Setpoint(desiredAngle);
        _tolerance = tolerance;
        _allowEnd = allowEnd;
        addRequirements(Arm.getInstance());

    }

    public ArmMoveAndStayAtAngle2(double desiredAngleArm1, double desiredAngleArm2, double tolerance, boolean allowEnd) {

        setArm1Setpoint(desiredAngleArm1);
        setArm2Setpoint(desiredAngleArm2);
        _tolerance = tolerance;
        _allowEnd = allowEnd;

        addRequirements(Arm.getInstance());

    }

    // CHANGE: remove initilize command
    public void setUpAndDown(boolean upAndDown) {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("arm1-setpoint", _arm1SetPoint);
        SmartDashboard.putNumber("arm2-setpoint", _arm2SetPoint);
        // CHANGE: removed, (1 - Chassis.getSpeedMagnitude()), probably will cause
        // unconsistent behviour, which will cause to the arm to move unprecisely to its
        // setpoint, and in diffrent why any time luke can low down when opening the arm
        // (if we want we might force him to prograematiclly)
        double output = (_kfArm1 * Math.sin(Math.toRadians(_arm1SetPoint)) +
                _arm1Kp * (_arm1SetPoint - Arm.getFirstAngle()));
        Arm.getFirst().set(output);
        // CHANGE: instead of checking whter first arm in tolerance of its target, since
        // what we actually want to check is whether the second arm will hit the
        // robot while opening i think that more logical test would be whether the
        // absoulte value of first arm is greator
        // then some value.
        // beware!! after moving the second arm there is more wheight on its edge, and
        // therfore more force is requierd, but if it dosent work on first try dont
        // abond it, maybe try to change kf after the arm is starting to open (i put
        // code in comment, becaues it may not be neede)
        if (Math.abs(Arm.getFirstAngle()) <= m_notHitAngle) { // goes to target when big arm withinsecond
                                                              // tolerance
            // _kfArm1*=1.3;
            double output2 = _kfArm2 * Math.sin(Math.toRadians(_arm2SetPoint)) +
                    _kpArm2 * (_arm2SetPoint * -1 * Math.signum(_arm1SetPoint) - Arm.getSecondAngle());
            Arm.getSecond().set(output2);
        }
        // CHANGE: remove part in comment below. unneccry, and probably caused second
        // arm to
        // move down

        // } else { // goes to 0 when big arm goes to target
        // double output2 = _kfArm2Down * Math.sin(Math.toRadians(_arm2SetPoint)) +
        // _kpArm2Down * (0 - Arm.getSecondAngle());
        // Arm.getSecond().set(output2);
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Arm.getFirst().set(_kfArm1 * Math.sin(Math.toRadians(_arm1SetPoint)));
    }

    public void setArm1Setpoint(double setPoint) {
        _arm1SetPoint = MathUtil.clamp(setPoint, Constants.ArmValues.FIRST_ARM_L_MIN,
                Constants.ArmValues.FIRST_ARM_R_MAX);
    }

    public void setArm2Setpoint(double setpoint) {

        _arm2SetPoint = MathUtil.clamp(setpoint, Constants.ArmValues.SECOND_ARM_MIN,
                Constants.ArmValues.SECOND_ARM_MAX);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(Arm.getFirstAngle() - _arm1SetPoint) <= _tolerance) && _allowEnd &&
                (Math.abs(Arm.getSecondAngle() - _arm2SetPoint) <= _tolerance);
    }
}

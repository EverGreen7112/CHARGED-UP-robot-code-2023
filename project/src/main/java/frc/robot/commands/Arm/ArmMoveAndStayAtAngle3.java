// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;

//changes mark as CHANGES, beware, probably wont work too well on back to 0,0 since while changing i wasent considerd this case
public class ArmMoveAndStayAtAngle3 extends CommandBase {
    private double _kfArm1 = 0.053, _arm1Kp = 0.0054, _kfArm2Down = 0.03, _kpArm2Down = 0.0047, m_notHitAngle = 40,
            m_constantDown = 0.5, _kfArm2Up = 0.03 * 1.5, _kpArm2Up = 0.0047 * 1.5, m_Kcontroller1 = 0.05,
            m_Kcontroller2 = 0.03;
    private double _arm1SetPoint, _arm2SetPoint, _tolerance;
    private boolean _allowEnd = true;
    private boolean m_isUpAndDown = false;
    private double m_output1 = 0;
    private double m_output2 = 0;
    private boolean m_humanControl = false;
    private int m_lastPov = -1;
    // CHANGE: added move up and down boolean that declares whthter the second arm
    // should move only down or up and the down, should probably only be true on
    // collection, added them in constructors as well

    /** Creates a new MoveAndStayAtAngle. */
    public ArmMoveAndStayAtAngle3(double desiredAngle, double tolerance, boolean upAndDown) {
        setArm1Setpoint(desiredAngle);
        _tolerance = tolerance;
        addRequirements(Arm.getInstance());
    }

    public ArmMoveAndStayAtAngle3(double desiredAngleArm1, double desiredAngleArm2, double tolerance,
            boolean upAndDown) {
        m_isUpAndDown = upAndDown;

        setArm1Setpoint(desiredAngleArm1);
        setArm2Setpoint(desiredAngleArm2);
        _tolerance = tolerance;
        addRequirements(Arm.getInstance());

        m_isUpAndDown = upAndDown;

    }

    public ArmMoveAndStayAtAngle3(double desiredAngle, double tolerance, boolean allowEnd, boolean upAndDown) {
        setArm1Setpoint(desiredAngle);
        _tolerance = tolerance;
        _allowEnd = allowEnd;
        addRequirements(Arm.getInstance());

        m_isUpAndDown = upAndDown;

    }

    public ArmMoveAndStayAtAngle3(double desiredAngleArm1, double desiredAngleArm2, double tolerance, boolean allowEnd,
            boolean upAndDown) {

        setArm1Setpoint(desiredAngleArm1);
        setArm2Setpoint(desiredAngleArm2);
        _tolerance = tolerance;
        _allowEnd = allowEnd;
        m_isUpAndDown = upAndDown;

        addRequirements(Arm.getInstance());

    }

    // CHANGE: remove initilize command
    public void setUpAndDown(boolean upAndDown) {
        m_isUpAndDown = upAndDown;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // CHANGE: added human control integrated inside, when pov is pressed last
        // output is saved and being added as pov
        if (!m_humanControl) {
            SmartDashboard.putNumber("arm1-setpoint", _arm1SetPoint);
            SmartDashboard.putNumber("arm2-setpoint", _arm2SetPoint);
            m_output1 = (_kfArm1 * Math.sin(Math.toRadians(_arm1SetPoint)) +
                    _arm1Kp * (_arm1SetPoint - Arm.getFirstAngle()));
            Arm.getFirst().set(m_output1);

            if (Math.abs(Arm.getFirstAngle()) <= m_notHitAngle) { // goes to target when big arm withinsecond
                                                                  // tolerance

                // CHANGE: added upp and down consideration if m_isUpAndDown is true
                if (!m_isUpAndDown) {
                    m_output2 = _kfArm2Down * Math.sin(Math.toRadians(_arm2SetPoint)) +
                            _kpArm2Down * (_arm2SetPoint * -1 * Math.signum(_arm1SetPoint) - Arm.getSecondAngle());
                } else {
                    if (Math.abs(Arm.getSecondAngle()) < 180 - Math.abs(Arm.getFirstAngle())) {
                        m_output2 = Math.signum(Arm.getFirstAngle()) * m_constantDown;// maybe multiply -1
                    } else {
                        m_output2 = _kfArm2Up * Math.sin(Math.toRadians(_arm2SetPoint)) +
                                _kpArm2Up * (_arm2SetPoint * -1 * Math.signum(_arm1SetPoint) - Arm.getSecondAngle());
                        Arm.getSecond().set(m_output2);
                    }
                }
                Arm.getSecond().set(m_output2);
                // if pov is pressed move to humancontrol mode and add to output
                if (RobotContainer.m_operator.getPOV() != -1) {
                    m_humanControl = true;
                    m_lastPov = RobotContainer.m_operator.getPOV();
                    switch (m_lastPov) {
                        case 0:
                            m_output1 += m_Kcontroller1;
                            break;
                        case 180:
                            m_output1 += m_Kcontroller1 * -1;
                            break;
                        case 90:
                            m_output2 += m_Kcontroller2;
                            break;
                        case 270:
                            m_output2 += m_Kcontroller2 * -1;
                            break;

                        default:
                            break;
                    }
                }
            }
            if (m_humanControl) {
                Arm.getFirst().set(m_output1);
                Arm.getSecond().set(m_output2);
                // if new other button then last pressed, ie presse and realse (since relaes
                // change to -1)
                if (RobotContainer.m_operator.getPOV() != m_lastPov) {
                    m_lastPov = RobotContainer.m_operator.getPOV();
                    if (RobotContainer.m_operator.getPOV() != -1) {
                        m_humanControl = true;
                        m_lastPov = RobotContainer.m_operator.getPOV();
                        switch (m_lastPov) {
                            case 0:
                                m_output1 += m_Kcontroller1;
                                break;
                            case 180:
                                m_output1 += m_Kcontroller1 * -1;
                                break;
                            case 90:
                                m_output2 += m_Kcontroller2;
                                break;
                            case 270:
                                m_output2 += m_Kcontroller2 * -1;
                                break;

                            default:
                                break;
                        }
                    }
                }
            }

        }
        // CHANGE: add power contro inside

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

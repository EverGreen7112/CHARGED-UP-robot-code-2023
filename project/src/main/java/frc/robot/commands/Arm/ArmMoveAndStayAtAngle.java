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

public class ArmMoveAndStayAtAngle extends CommandBase {
  private double _kfArm1 = 0.05, _arm1Kp = 0.005, _kfArm2 = 0.03, _kpArm2 = 0.006;
  private double _arm1SetPoint, _arm2SetPoint, _tolerance;
  private boolean _allowEnd = true;
  
  /** Creates a new MoveAndStayAtAngle. */
  public ArmMoveAndStayAtAngle(double desiredAngle, double tolerance) {
    addRequirements(Arm.getInstance());
    setArm1Setpoint(desiredAngle);
    _tolerance = tolerance;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public ArmMoveAndStayAtAngle(double desiredAngleArm1, double desiredAngleArm2, double tolerance) {
    addRequirements(Arm.getInstance());
    setArm1Setpoint(desiredAngleArm1);
    setArm2Setpoint(desiredAngleArm2);
    _tolerance = tolerance;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public ArmMoveAndStayAtAngle(double desiredAngle, double tolerance, boolean allowEnd) {
    addRequirements(Arm.getInstance());
    setArm1Setpoint(desiredAngle);
    _tolerance = tolerance;
    _allowEnd = allowEnd;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public ArmMoveAndStayAtAngle(double desiredAngleArm1, double desiredAngleArm2, double tolerance, boolean allowEnd) {
    addRequirements(Arm.getInstance());
    setArm1Setpoint(desiredAngleArm1);
    setArm2Setpoint(desiredAngleArm2);
    _tolerance = tolerance;
    _allowEnd = allowEnd;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("arm-kf", 0.05);
    // SmartDashboard.putNumber("arm-kp", 0.001);
    // SmartDashboard.putNumber("arm2-kf", 0.0);
    // SmartDashboard.putNumber("arm2-kp", 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // _pid.setP(SmartDashboard.getNumber("arm-kp", 0));
    // double kp = SmartDashboard.getNumber("arm-kp", 0);
    // _kfArm1 = SmartDashboard.getNumber("arm-kf", 0);
    double output = (_kfArm1 * Math.sin(Math.toRadians(_arm1SetPoint)) +
    _arm1Kp * (_arm1SetPoint - Arm.getFirstAngle())) * (1 - Chassis.getSpeedMagnitude());
    Arm.getFirst().set(output);
    if (Math.abs(Arm.getFirstAngle() - _arm1SetPoint) <= _tolerance) { // goes to target when big arm within tolerance
      double output2 = _kfArm2 * Math.sin(Math.toRadians(_arm2SetPoint)) +
      _kpArm2 * (_arm2SetPoint * -1 * Math.signum(_arm1SetPoint) - Arm.getSecondAngle());
      Arm.getSecond().set(output2);
    }
    else { // goes to 0 when big arm goes to target
      // _kpArm2 = SmartDashboard.getNumber("arm-kp", 0);
      // _kfArm2 = SmartDashboard.getNumber("arm-kf", 0);
      double output2 = _kfArm2 * Math.sin(Math.toRadians(_arm2SetPoint)) +
      _kpArm2 * (0 - Arm.getSecondAngle());
      Arm.getSecond().set(output2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.getFirst().set(_kfArm1 * Math.sin(Math.toRadians(_arm1SetPoint)));
  }

  public void setArm1Setpoint(double setPoint){
    // _setPoint = MathUtil.clamp(setPoint, Constants.ArmValues.FIRST_ARM_L_MIN, Constants.ArmValues.FIRST_ARM_R_MAX);
    _arm1SetPoint = MathUtil.clamp(setPoint, Constants.ArmValues.FIRST_ARM_L_MIN,
     Constants.ArmValues.FIRST_ARM_R_MAX);
  }
  public void setArm2Setpoint(double setpoint) {

    _arm2SetPoint = MathUtil.clamp(setpoint, Constants.ArmValues.SECOND_ARM_MIN, Constants.ArmValues.SECOND_ARM_MAX);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(Arm.getFirstAngle() - _arm1SetPoint) <= _tolerance) && _allowEnd && 
    (Math.abs(Arm.getSecondAngle() - _arm2SetPoint) <= _tolerance);
  }
}

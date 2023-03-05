// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision;
import frc.robot.subsystems.Chassis;

public class DriveDistanceByVision extends CommandBase {
  /** Creates a new DriveDistanceByReflector. */
  private Vision _vision;
  private Chassis chassis = Chassis.getInstance();
  PIDController _pidZ, _pidX;
  private double _targetDistance, _targetX, _kf, _tolerance, _feedForward;
  public DriveDistanceByVision(Double distance, Vision vision, double kf, double kp, double ki, double kd, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Chassis.getInstance());
    _vision = vision;
    _targetDistance = distance;
    // _kp = kp;
    // _ki = ki;
    // _kd = kd;
    _kf = kf;
    _tolerance = tolerance;
    _pidZ = new PIDController(kp, ki, kd);
    _pidX = new PIDController(kp, ki, kd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      _feedForward = _targetDistance * _kf;
      chassis.driveTank(_feedForward, _feedForward);
      _targetDistance += _vision.getZ();
      _targetX = _vision.getX();
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double l = _feedForward;
    double r = _feedForward;
    double eX = _pidX.calculate(_vision.getX(), _targetX);
    l += eX;
    r -= eX;
    double eZ = _pidZ.calculate(_vision.getZ(), _targetDistance);
    l += eZ;
    r += eZ;
    chassis.driveTank(l, r);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.driveTank(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(_vision.getZ() - _targetDistance) <= _tolerance;
  }
}

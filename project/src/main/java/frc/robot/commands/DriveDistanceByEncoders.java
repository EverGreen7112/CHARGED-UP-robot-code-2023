// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vector2D;
import frc.robot.subsystems.Chassis;

public class DriveDistanceByEncoders extends CommandBase {
  private double _targetDistance;
  private double _distance, _feedForward = 0;
  private PIDController _distancePID;
  private PIDController _anglePID;
  /**
   * @param distance the distance to drive in meters 
   * @param distanceTolerance the error tolerance for the PID in meters
   */
  public DriveDistanceByEncoders(double distance, double distanceTolerance) {
    addRequirements(Chassis.getInstance());
    _distance = distance;
    _distancePID = new PIDController(Constants.PIDS.driveKp, Constants.PIDS.driveKi, Constants.PIDS.driveKd);
    _distancePID.setTolerance(distanceTolerance);

    _anglePID = new PIDController(Constants.PIDS.rotateKp, Constants.PIDS.rotateKi, Constants.PIDS.rotateKd);
  }
  /**
   * @param distance the distance to drive in meters 
   * @param tolerance the error tolerance for the PID in meters
   */
  public DriveDistanceByEncoders(double distance, double tolerance, double kf) {
    addRequirements(Chassis.getInstance());
    _distance = distance;
    _distancePID = new PIDController(Constants.PIDS.driveKp, Constants.PIDS.driveKi, Constants.PIDS.driveKd);
    _feedForward = kf * distance;
    _distancePID.setTolerance(tolerance);

    _anglePID = new PIDController(Constants.PIDS.rotateKp, Constants.PIDS.rotateKi, Constants.PIDS.rotateKd);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _targetDistance = Chassis.getEncodersDist() + _distance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("distance", Chassis.getEncodersDist() - (_targetDistance - _distance));
    SmartDashboard.putNumber("angle: ", Chassis.getRobotAngle());
    double distanceSpeed = _feedForward + (_distancePID.calculate(Chassis.getEncodersDist(), _targetDistance));
    double angleSpeed = _anglePID.calculate(Chassis.getRobotAngle(), 0);
    Vector2D v = new Vector2D(MathUtil.clamp(distanceSpeed - angleSpeed, -1.0, 1.0), MathUtil.clamp(distanceSpeed + angleSpeed, -1.0, 1.0));
    v.normalize();
    v.multiply(Math.abs(distanceSpeed) * Math.sqrt(2));
    // Chassis.driveTank(v.x * (Math.signum(_distance)), v.y  * (Math.signum(_distance)));
    Chassis.driveTank(v.x , v.y );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Chassis.getInstance().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _distancePID.atSetpoint();
  }
}

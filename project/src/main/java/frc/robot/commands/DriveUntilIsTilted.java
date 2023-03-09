package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vector2D;
import frc.robot.subsystems.Chassis;

public class DriveUntilIsTilted extends CommandBase{
    
    PIDController _anglePID;
    double _initYaw;

    @Override
    public void initialize() {
        _initYaw = Chassis.getGyro().getAngle();
        _anglePID = new PIDController(Constants.PIDS.rotateKp, Constants.PIDS.rotateKi, Constants.PIDS.rotateKd);
    }
    @Override
    public void execute() {
        double speed = 0.5;
        double angleSpeed = _anglePID.calculate(Chassis.getGyro().getAngle(), _initYaw);
        Vector2D v = new Vector2D(speed + angleSpeed, speed - angleSpeed);
        v.normalize();
        Chassis.driveTank(v.x * Math.abs(speed), v.y * Math.abs(speed));
    }
    @Override
    public boolean isFinished() {
        return Math.abs(Chassis.getGyro().getRoll()) > 7;
    }
}

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vector2D;
import frc.robot.subsystems.Chassis;

public class Balance extends CommandBase{
    
    private double m_gyroValue;
    private double kp = 0.11/16;
    private PIDController _anglePID;
    private double _initYaw;
    @Override
    public void initialize() {
        _anglePID = new PIDController(Constants.PIDS.rotateKp, Constants.PIDS.rotateKi, Constants.PIDS.rotateKd);
        _initYaw = Chassis.getGyro().getAngle();
        // Chassis.setMode(IdleMode.kCoast);
        Chassis.setMode(IdleMode.kBrake);
    }
    @Override
    public void execute() {
        if(Math.abs(Chassis.getGyro().getRoll()) <= Constants.Values.BALANCE_COMMAND_TOLERANCE){
            Chassis.stop();
        }
        else{
            // Chassis.setMode(IdleMode.kCoast);
            m_gyroValue = Chassis.getGyro().getRoll();
            double distanceSpeed = kp * m_gyroValue;
        double angleSpeed = _anglePID.calculate(Chassis.getGyro().getAngle(), _initYaw);
        Vector2D v = new Vector2D( distanceSpeed - angleSpeed, distanceSpeed + angleSpeed);
        v.normalize();
        Chassis.driveTank(MathUtil.clamp(Math.abs(v.x) * Math.sqrt(2)* distanceSpeed, -12 * kp, 12 * kp),
        MathUtil.clamp(Math.abs(v.y) * Math.sqrt(2) *  distanceSpeed, -12 * kp, 12 * kp));
        }
        
        // SmartDashboard.putNumber("gyro value", m_gyroValue);
    }

    public boolean isFinished(){
   //     return 
        return false;
}
    @Override
    public void end(boolean interrupted) {
        // Chassis.setMode(IdleMode.kCoast);
        Chassis.stop();
    }

}

package frc.robot.commands.Arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm1ByAngle extends CommandBase{
    private PIDController cont = new PIDController(0, 0, 0);
    public MoveArm1ByAngle(double desierdAngle) {
        addRequirements(Arm.getInstance());
        cont.setSetpoint(desierdAngle);
    }
    @Override
    public void initialize() {
        double [] fpid = Arm.getFirstFPID();
        cont.setPID(fpid[1], fpid[2], fpid[3]);
        cont.setTolerance(Arm.firstArmTol,Arm.firstArmVTol);
    }
    private double lastOutput = 0;
    @Override
    public void execute() {
        double [] fpid = Arm.getFirstFPID();
        //might use othe more physiclly accurate calculation 
        lastOutput = fpid[0]*Math.signum(fpid[0]) + cont.calculate(Arm.getFirstAngle());
        Arm.getFirst().set(TalonFXControlMode.PercentOutput,lastOutput);
    }
    @Override
    public boolean isFinished() {
        return cont.atSetpoint();
    }
    @Override
    public void end(boolean interrupted) {
        
    }
}

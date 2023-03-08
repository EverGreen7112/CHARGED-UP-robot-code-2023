package frc.robot.commands.Arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm2ByAngle extends CommandBase {
    private PIDController cont = new PIDController(0, 0, 0);
    private double m_desierdAngle;
    public MoveArm2ByAngle(double desierdAngle) {
        addRequirements(Arm.getInstance());
        m_desierdAngle = desierdAngle;
        cont.setSetpoint(desierdAngle);
    }

    @Override
    public void initialize() {
        double[] fpid = Arm.getSecondUpFPID();
        cont.setPID(fpid[1], fpid[2], fpid[3]);
        cont.setTolerance(Arm.firstArmTol, Arm.firstArmVTol);
    }

    private double lastOutput = 0;

    @Override
    public void execute() {
        ex1();
        // ex2();
    }

    // trying approch in which there is a distinction between moving the arm down
    // (i.e with gravity) and moving the arm up (i.e against gravity)
    public void ex1(){
        double[] fpid = Arm.getSecondUpFPID();
        if(180-Math.abs(Arm.getFirstAngle()) < Math.abs(m_desierdAngle)){
            lastOutput = fpid[0] * Math.signum(fpid[0]) + cont.calculate(Arm.getFirstAngle());
            
        }
    }

    // normal pid
    public void ex2() {
        double[] fpid = Arm.getSecondUpFPID();
        // might use othe more physiclly accurate calculation
        lastOutput = fpid[0] * Math.signum(fpid[0]) + cont.calculate(Arm.getFirstAngle());
        Arm.getSecond().set(TalonFXControlMode.PercentOutput, lastOutput);
    }

    @Override
    public boolean isFinished() {
        return cont.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

    }
}

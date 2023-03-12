package frc.robot.commands.Arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArm1ByAngle extends CommandBase{
    private PIDController cont = new PIDController(0, 0, 0);
    private Supplier<Double> m_desAng;
    private boolean m_stallSecond = false;

    public MoveArm1ByAngle(Supplier<Double> desierdAngle) {
        addRequirements(Arm.getInstance());
        m_desAng = desierdAngle;
        cont.setSetpoint(desierdAngle.get());
    }
    public MoveArm1ByAngle(Supplier<Double> desierdAngle, boolean stalSec) {
        addRequirements(Arm.getInstance());
        m_desAng = desierdAngle;
        cont.setSetpoint(desierdAngle.get());
        m_stallSecond = stalSec;
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
        cont.setSetpoint(m_desAng.get());
        ex2();
        if(m_stallSecond){
            Arm.stall2();
        }
    }
    //normal pid
    private void ex1(){
        double [] fpid = Arm.getFirstFPID();
        lastOutput = fpid[0]*Math.signum(fpid[0]) + cont.calculate(Arm.getFirstAngle());
        Arm.getFirst().set(lastOutput);
    }
    //physical calculation pid
    private void ex2(){
        double [] fpid = Arm.getFirstFPID();
        lastOutput = fpid[0]*Math.sin(Math.toRadians(m_desAng.get())) + cont.calculate(Arm.getFirstAngle());
        Arm.getFirst().set(lastOutput);
    }
    @Override
    public boolean isFinished() {
        return cont.atSetpoint();
    }

}

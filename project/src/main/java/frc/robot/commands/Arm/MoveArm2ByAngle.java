package frc.robot.commands.Arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArm2ByAngle extends CommandBase {
    private PIDController cont = new PIDController(0, 0, 0);
    private double m_desierdAngle;
    private CANSparkMax m_second;
    private boolean m_stallFirst = false;
    private StallArm1 sta1;
    public MoveArm2ByAngle(double desierdAngle) {
        addRequirements(Arm.getInstance());
        m_desierdAngle = desierdAngle;
        cont.setSetpoint(desierdAngle);
        m_second = Arm.getSecond();
        
    }
    public MoveArm2ByAngle(double desierdAngle,boolean stallFirst) {
        addRequirements(Arm.getInstance());
        m_desierdAngle = desierdAngle;
        cont.setSetpoint(desierdAngle);
        m_second = Arm.getSecond();
        m_stallFirst = stallFirst;
    }

    @Override
    public void initialize() {
        double[] fpid = Arm.getSecondUpFPID();
        cont.setPID(fpid[1], fpid[2], fpid[3]);
        cont.setTolerance(Arm.firstArmTol, Arm.firstArmVTol);
        sta1 = new StallArm1();
    }

    private double lastOutput = 0;

    @Override
    public void execute() {
        ex1();
        if(m_stallFirst){
            Arm.stall1();
        }
        // ex2();
    }

    // trying approch in which there is a distinction between moving the arm down
    // (i.e with gravity) and moving the arm up (i.e against gravity)
    public void ex1(){
        double[] fpid = Arm.getSecondUpFPID();
        if(180-Math.abs(Arm.getFirstAngle()) < Math.abs(m_desierdAngle)){
            lastOutput = fpid[0] * Math.signum(fpid[0]) + cont.calculate(Arm.getFirstAngle());
            if(lastOutput < 0){
                m_second.set(Constants.Speeds.ARM2SPEED);
            }
            else{
                
            }
        }
        else{

        }
    }
    //physical calculation pid
    private void ex2(){
        double [] fpid = Arm.getFirstFPID();
        lastOutput = fpid[0]*Math.sin(Math.toRadians(Arm.getFirstAngle())) + cont.calculate(Arm.getFirstAngle());
        Arm.getFirst().set(lastOutput);
    }
    // normal pid
    public void ex3() {
        double[] fpid = Arm.getSecondUpFPID();
        // might use othe more physiclly accurate calculation
        lastOutput = fpid[0] * Math.signum(fpid[0]) + cont.calculate(Arm.getFirstAngle());
        Arm.getSecond().set(lastOutput);        }
    @Override
    public boolean isFinished() {
        return cont.atSetpoint();
    }


}

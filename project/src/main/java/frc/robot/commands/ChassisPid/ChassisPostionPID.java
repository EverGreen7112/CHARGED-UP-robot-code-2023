package frc.robot.commands.ChassisPid;


import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDS;
import frc.robot.subsystems.Chassis;

/**
 * Command for chassi postion pid using inner sparkmax pid loop
 */
public class ChassisPostionPID extends CommandBase {

    protected DoubleSupplier m_setPoint;
    protected double m_startDist =-1;
    public ChassisPostionPID(DoubleSupplier setpointSource) {
        m_setPoint = setpointSource;
        SparkMaxPIDController rpid = Chassis.getInstance().getRightPID();
        SparkMaxPIDController lpid = Chassis.getInstance().getLeftPID();
        addRequirements(Chassis.getInstance());

    }

    @Override
    public void initialize() {
        Chassis.getInstance().getRightPID().setIAccum(0);
        Chassis.getInstance().getLeftPID().setIAccum(0);
        Chassis.getInstance().getRightPID().setReference(m_setPoint.getAsDouble(),CANSparkMax.ControlType.kPosition
        , 0);
        Chassis.getInstance().getLeftPID().setReference(m_setPoint.getAsDouble(),CANSparkMax.ControlType.kPosition
        , 0);
        m_startDist = Chassis.getInstance().getEncodersDist();
    }

    public double getCurDistance(){
        return Chassis.getInstance().getEncodersDist()-m_startDist;
    }
    @Override
    public void end(boolean interrupted) {
        Chassis.getInstance().stop();
    }
}

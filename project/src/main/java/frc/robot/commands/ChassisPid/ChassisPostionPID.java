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
        SparkMaxPIDController rpid = Chassis.getRightPID();
        SparkMaxPIDController lpid = Chassis.getLeftPID();
        addRequirements(Chassis.getInstance());

    }

    @Override
    public void initialize() {
        Chassis.getRightPID().setIAccum(0);
        Chassis.getLeftPID().setIAccum(0);
        Chassis.getRightPID().setReference(m_setPoint.getAsDouble(),CANSparkMax.ControlType.kPosition
        , 0);
        Chassis.getLeftPID().setReference(m_setPoint.getAsDouble(),CANSparkMax.ControlType.kPosition
        , 0);
        m_startDist = Chassis.getEncodersDist();
    }

    public double getCurDistance(){
        return Chassis.getEncodersDist()-m_startDist;
    }
    @Override
    public void end(boolean interrupted) {
        Chassis.stop();
    }
}

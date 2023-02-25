package frc.robot.commands.ChassisPid;

import java.sql.Time;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Chassis;

public class ChassisTimePosPID extends ChassisPostionPID{
    private Timer m_timer;
    private Supplier<Double> m_finishTime;

    public ChassisTimePosPID(DoubleSupplier setpointSource,Supplier<Double> finishTime) {
        super(setpointSource);
        m_finishTime=finishTime;

    }
    @Override
    public void initialize() {
        m_timer = new Timer();
        super.initialize();
    }
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_finishTime.get());

    }
}

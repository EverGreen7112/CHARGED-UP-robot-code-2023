package frc.robot.commands.General;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 *  * Same as PIDCommand but ends when constant time has elapsed

 */
public class PIDTimedCommand extends PIDCommand {
    private Timer m_timer;
    private Supplier<Double> m_finishTime;

    public PIDTimedCommand(PIDController controller, DoubleSupplier measurementSource, double setpoint,
            DoubleConsumer useOutput, Subsystem[] requirements, Supplier<Double> finishTime) {
        super(controller, measurementSource, setpoint, useOutput, requirements);
        m_finishTime = finishTime;
    }

    public PIDTimedCommand(PIDController controller, DoubleSupplier measurementSource, DoubleSupplier setpointSource,
            DoubleConsumer useOutput, Supplier<Double> finishTime, Subsystem... requirements) {
        super(controller, measurementSource, setpointSource, useOutput, requirements);
        m_finishTime = finishTime;

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

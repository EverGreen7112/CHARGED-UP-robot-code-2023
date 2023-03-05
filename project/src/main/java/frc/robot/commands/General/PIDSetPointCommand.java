package frc.robot.commands.General;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Same as PIDCommand but ends when PIDcontroller at setPoint
 */
public class PIDSetPointCommand extends PIDCommand {

    public PIDSetPointCommand(PIDController controller, DoubleSupplier measurementSource, double setpoint,
            DoubleConsumer useOutput, Subsystem... requirements) {
        super(controller, measurementSource, setpoint, useOutput, requirements);
    }

    public PIDSetPointCommand(PIDController controller, DoubleSupplier measurementSource, DoubleSupplier setpointSource,
            DoubleConsumer useOutput, Subsystem... requirements) {
        super(controller, measurementSource, setpointSource, useOutput, requirements);
    }

    @Override
    public boolean isFinished() {
        m_controller.setSetpoint(m_setpoint.getAsDouble());
        return m_controller.atSetpoint();
    }
}

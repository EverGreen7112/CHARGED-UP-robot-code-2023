package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArmBySupllier extends CommandBase {

    Supplier<Double> value1, value2;
    TalonFX m_motor;
    double m_JoystickAngle;

    public MoveArmBySupllier(Supplier<Double> a, Supplier<Double> b, int mode) {
        addRequirements(Arm.getInstance());
        value1 = a;
        value2 = b;
        if (mode == 1){
            m_motor = Arm.getInstance().getFirst();
        } else if (mode == 2){
            m_motor = Arm.getInstance().getSecond();
        }
    }

    @Override
    public void initialize() {
        // m_motor = new TalonFX(Constants.Ports.FIRST_ARM_PORT);
        // m_motor.configFactoryDefault();
        // m_motor.selectProfileSlot(0, 0);
        // m_motor.config_kP(0, 0.03);
        // m_motor.config_kI(0, 0);
        // m_motor.config_kD(0, 0);
        // m_motor.setSensorPhase(true);
        // m_motor.setSelectedSensorPosition(0);
        m_JoystickAngle = 0;
    }

    @Override
    public void execute() {
        if (Math.abs(value1.get()) < Constants.ArmValues.JOYSTICK_TOLERANCE && Math.abs(value2.get()) < Constants.ArmValues.JOYSTICK_TOLERANCE) {
            return;
        }
        m_JoystickAngle = Math.toDegrees(Math.atan2(value1.get(), value2.get()));
        if (m_JoystickAngle <= 0)
            m_JoystickAngle += 360;
        double m_armAngle = Constants.Conversions.ticksToAngle(m_motor.getSelectedSensorPosition());
        double m_armTarget = Constants.Conversions.angleToTicks(m_armAngle + Constants.Conversions.closestAngle(m_armAngle, m_JoystickAngle));
        // if (m_JoystickAngle <= Constants.ArmValues.FIRST_ARM_MAX && m_JoystickAngle >= Constants.ArmValues.FIRST_ARM_MIN) {
            // if (Math.abs(m_JoystickAngle - Math.abs(Constants.Conversions.modulo(Constants.Conversions.ticksToAngle(m_armTarget), 360))) > Constants.ArmValues.FIRST_ARM_MAX + 5 || Math.abs(m_JoystickAngle - Math.abs(Constants.Conversions.modulo(Constants.Conversions.ticksToAngle(m_armTarget), 360))) < Constants.ArmValues.FIRST_ARM_MIN - 5) {
                // motor.set(TalonSRXControlMode.Position, m_armTarget);
            // } 
            // else {
        m_motor.set(TalonFXControlMode.Position, m_armTarget);
            // }
        // }
        SmartDashboard.putNumber("joystick angle", m_JoystickAngle);
        SmartDashboard.putNumber("curr angle", m_armAngle);
        SmartDashboard.putNumber("target", m_armTarget);
        SmartDashboard.putNumber("position", m_motor.getSelectedSensorPosition());
        SmartDashboard.putNumber("output", m_motor.getMotorOutputPercent());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

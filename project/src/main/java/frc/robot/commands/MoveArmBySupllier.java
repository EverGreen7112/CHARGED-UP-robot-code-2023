package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArmBySupllier extends CommandBase {

    Supplier<Double> value1, value2;
    TalonFX m_motor;
    double m_JoystickAngle, m_armAngle, m_armTarget;
    int m_mode;
    DigitalInput a = new DigitalInput(53);

    //mode controls which arm the command moves, 1 moves the first arm, 2 moves the second arm.
    public MoveArmBySupllier(Supplier<Double> x, Supplier<Double> y, int mode) {
        addRequirements(Arm.getInstance());
        value1 = x;
        value2 = y;
        m_mode = mode;
        if (mode == 1){
            m_motor = Arm.getInstance().getFirst();
        } else if (mode == 2){
            m_motor = Arm.getInstance().getSecond();
        }
    }

    @Override
    public void initialize() {
        m_JoystickAngle = 0;
    }
    
    @Override
    public void execute() {
        //tolerance for joystick.
        if (Math.abs(value1.get()) < Constants.ArmValues.JOYSTICK_TOLERANCE && Math.abs(value2.get()) < Constants.ArmValues.JOYSTICK_TOLERANCE) {
            return;
        }
        //convert joystick values to degrees.
        m_JoystickAngle = Math.toDegrees(Math.atan2(value1.get(), value2.get()));
        if (m_JoystickAngle <= 0)
            m_JoystickAngle += 360;
        if (m_mode == 1){
            m_armAngle = Constants.Conversions.ticksToAngle(m_motor.getSelectedSensorPosition(), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
            m_armTarget = Constants.Conversions.angleToTicks(m_armAngle + Constants.Conversions.closestAngle(m_armAngle, m_JoystickAngle), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);    
        } else if (m_mode == 2){
            m_armAngle = Constants.Conversions.ticksToAngle(m_motor.getSelectedSensorPosition(), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
            m_armTarget = Constants.Conversions.angleToTicks(m_armAngle + Constants.Conversions.closestAngle(m_armAngle, m_JoystickAngle), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
        }
        
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

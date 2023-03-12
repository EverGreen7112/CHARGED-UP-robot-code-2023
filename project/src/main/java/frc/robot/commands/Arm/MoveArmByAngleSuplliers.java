package frc.robot.commands.Arm;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArmByAngleSuplliers extends CommandBase {
    Supplier<Double> value1, value2;
    CANSparkMax m_motor;
    double m_JoystickAngle, m_armAngle, m_armTarget;
    int m_mode;

    //mode controls which arm the command moves, 1 moves the first arm, 2 moves the second arm.
    public MoveArmByAngleSuplliers(Supplier<Double> x, Supplier<Double> y, int mode) {
        //  addRequirements(Arm.getInstance());
        value1 = x;
        value2 = y;
        m_mode = mode;
        if (mode == 1){
            m_motor = Arm.getFirst();
        } else if (mode == 2){
            m_motor = Arm.getSecond();
        }
    }

    @Override
    public void initialize() {
        m_JoystickAngle = 0;
    }
    
    @Override
    public void execute() {
        //tolerance for joystick.
        if (Math.abs(value1.get()) < Constants.ArmValues.JOYSTICK_ANGLE_TOLERANCE && Math.abs(value2.get()) < Constants.ArmValues.JOYSTICK_ANGLE_TOLERANCE) {
            return;
        }
        //convert joystick values to degrees.
        m_JoystickAngle = Math.toDegrees(Math.atan2(value1.get(), value2.get()));
        if (m_JoystickAngle <= 0)
            m_JoystickAngle += 360;
        if (m_mode == 1){
            // m_armAngle = Arm.getFirstAngle();
            // m_armTarget = Constants.Conversions.angleToTicks(m_armAngle + Constants.Conversions.closestAngle(m_armAngle, m_JoystickAngle), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);    
            new MoveArm1ByAngle(()->m_JoystickAngle).schedule();
        } else if (m_mode == 2){
            // m_armAngle = Arm.getSecondAngle();
            // m_armTarget = Constants.Conversions.angleToTicks(m_armAngle + Constants.Conversions.closestAngle(m_armAngle, m_JoystickAngle), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
            new MoveArm2ByAngle(()->m_JoystickAngle).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

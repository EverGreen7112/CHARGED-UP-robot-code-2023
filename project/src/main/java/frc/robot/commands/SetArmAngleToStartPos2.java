package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PidValues;
import frc.robot.subsystems.Arm;

public class SetArmAngleToStartPos2 extends CommandBase {
    TalonFX m_first;
    TalonFX m_second;
    double m_firstArmAngle;
    double m_secondArmAngle;

    public SetArmAngleToStartPos2() {
        addRequirements(Arm.getInstance());
        m_first = Arm.getInstance().getFirst();
        m_second = Arm.getInstance().getSecond();

    }

    @Override
    public void initialize() {
        // m_first.config_kP(0, Constants.PidValues.FIRST_ARM_KP * 1.6);
        // m_first.config_kF(0, 0.35);
    }

    @Override
    public void execute() {

        m_firstArmAngle = Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(),
                Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
        m_secondArmAngle = Constants.Conversions.ticksToAngle(m_second.getSelectedSensorPosition(),
                Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);

        SmartDashboard.putBoolean("enterIf", m_secondArmAngle < Constants.ArmValues.LIMIT_TOLERANCE + 0
                && m_secondArmAngle > 0 - Constants.ArmValues.LIMIT_TOLERANCE);
        // if(m_secondArmAngle < Constants.ArmValues.LIMIT_TOLERANCE + 0 &&
        // m_secondArmAngle > 0 - Constants.ArmValues.LIMIT_TOLERANCE){
        // m_first.set(TalonFXControlMode.Position,
        // Constants.Conversions.angleToTicks(-1,
        // Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION));
        
        // if (m_firstArmAngle < 15 && m_firstArmAngle > -15) {
        //     if (m_firstArmAngle > 0) {
        //         m_first.set(TalonFXControlMode.PercentOutput, -PidValues.FIRST_ARM_ANTI_KF);
        //     } else {
        //         m_first.set(TalonFXControlMode.PercentOutput, PidValues.FIRST_ARM_ANTI_KF);

        //     }
        // }
        if (m_firstArmAngle > 0) {
            // m_first.set(TalonFXControlMode.PercentOutput,-PidValues.FIRST_ARM_ANTI_KF+
            // -1*Constants.PidValues.FIRST_ARM_ANTI_KP*(Constants.ArmValues.FIRST_ARM_R_MAX-m_firstArmAngle));
            // m_first.set(TalonFXControlMode.PercentOutput,-PidValues.FIRST_ARM_ANTI_KF);
            m_first.set(TalonFXControlMode.PercentOutput, -1 * Constants.PidValues.FIRST_ARM_ANTI_KP
                    * (Constants.ArmValues.FIRST_ARM_R_MAX - m_firstArmAngle));
        } else {
            m_first.set(TalonFXControlMode.PercentOutput, PidValues.FIRST_ARM_ANTI_KF + -1
                    * Constants.PidValues.FIRST_ARM_ANTI_KP * (Constants.ArmValues.FIRST_ARM_L_MIN - m_firstArmAngle));
            // m_first.set(TalonFXControlMode.PercentOutput,PidValues.FIRST_ARM_ANTI_KF);
            // m_first.set(TalonFXControlMode.PercentOutput,-1*Constants.PidValues.FIRST_ARM_ANTI_KP*(Constants.ArmValues.FIRST_ARM_L_MIN-m_firstArmAngle));

        }
        SmartDashboard.putNumber("test124", m_first.getMotorOutputPercent());
        // }else{
        // m_second.set(TalonFXControlMode.Position,
        // Constants.Conversions.angleToTicks(-1,
        // Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION));
        // }

    }

    @Override
    public boolean isFinished() {
        return m_firstArmAngle < 4 && m_firstArmAngle > -4;
    }

    @Override
    public void end(boolean interrupted) {
        m_first.config_kP(0, Constants.PidValues.FIRST_ARM_KP);
        m_first.config_kF(0, 0);
    }
}

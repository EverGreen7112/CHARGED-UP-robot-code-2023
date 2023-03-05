package frc.robot.commands.Arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PidValues;
import frc.robot.subsystems.Arm;

public class SetArmAngleToStartPos extends CommandBase {
    TalonFX m_first;
    TalonFX m_second;
    double m_firstArmAngle;
    double m_secondArmAngle;
    double startDir = 1;
    boolean cross = false;

    public SetArmAngleToStartPos() {
        addRequirements(Arm.getInstance());
        m_first = Arm.getFirst();
        m_second = Arm.getSecond();
    }

    boolean hardDir = false;

    @Override
    public void initialize() {
        m_second.config_kP(0, 0.035);
        startDir = Math.signum(Arm.getFirstAngle());
        m_second.config_kF(0, -0.7 * Math.signum(Arm.getFirstAngle()));
        m_second.config_kD(0, 0.0003);
        m_second.config_kI(0, 0);
        m_first.config_kF(0, 0);
        m_first.config_kP(0, PidValues.FIRST_ARM_BACK_KP * -3);
        m_first.config_kI(0, PidValues.FIRST_ARM_BACK_KI);
        m_first.config_kD(0, PidValues.FIRST_ARM_BACK_KD);
        m_firstArmAngle = Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(),
                Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
        hardDir = m_firstArmAngle > 0;
        endd = false;
    }

    private boolean endd;
    // private int stage = 0;
    boolean stage2 = false;

    @Override
    public void execute() {
        m_firstArmAngle = Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(),
                Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
        m_secondArmAngle = Constants.Conversions.ticksToAngle(m_second.getSelectedSensorPosition(),
                Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);

        m_second.set(TalonFXControlMode.Position,
                Constants.Conversions.angleToTicks(-1,
                        Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION));


        if (m_secondArmAngle < Constants.ArmValues.LIMIT_TOLERANCE
                && m_secondArmAngle > -1 * Constants.ArmValues.LIMIT_TOLERANCE) {
            m_first.set(TalonFXControlMode.Position, 0);
            if (m_firstArmAngle < 23 && m_firstArmAngle > -23) {
                SmartDashboard.putBoolean("isInRange", true);

                m_first.set(TalonFXControlMode.Position, 0);

            } 
            else {
                SmartDashboard.putBoolean("isInRange", false);
                if (m_firstArmAngle > 0) {
                    m_first.set(TalonFXControlMode.PercentOutput, -PidValues.FIRST_ARM_ANTI_KF +
                            -1 * Constants.PidValues.FIRST_ARM_ANTI_KP
                                    * (Constants.ArmValues.FIRST_ARM_R_MAX - m_firstArmAngle));
                    
                }
                else {
                    m_first.set(TalonFXControlMode.PercentOutput, PidValues.FIRST_ARM_ANTI_KF + -1
                            * Constants.PidValues.FIRST_ARM_ANTI_KP
                            * (Constants.ArmValues.FIRST_ARM_L_MIN - m_firstArmAngle));

                }
            }


        }
    }

    // @Override
    // public boolean isFinished() {
    // return m_firstArmAngle < 8 && m_firstArmAngle > -8;
    // }

    // @Override
    // public void end(boolean interrupted) {
    // endd = true;
    // SmartDashboard.putBoolean("endddd", endd)
    // m_first.config_kP(0, Constants.PidValues.FIRST_ARM_KP);
    // m_first.config_kF(0, 0);
    // }
}

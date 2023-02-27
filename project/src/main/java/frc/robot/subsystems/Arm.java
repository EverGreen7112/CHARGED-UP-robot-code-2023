package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private TalonFX m_first, m_second;
    private double m_firstMinRange, m_firstMaxRange, m_secondMinRange, m_secondMaxRange;
    private static Arm m_instance;

    public Arm(TalonFX first, TalonFX second) {
        m_first = first;
        m_firstMinRange = Constants.ArmValues.FIRST_ARM_MIN;
        m_firstMaxRange = Constants.ArmValues.FIRST_ARM_MAX;
        first.configFactoryDefault();
        first.selectProfileSlot(0, 0);
        first.config_kP(0, Constants.PidValues.FIRST_ARM_KP);
        first.config_kI(0, Constants.PidValues.FIRST_ARM_KI);
        first.config_kD(0, Constants.PidValues.FIRST_ARM_KD);
        first.setSensorPhase(true);
        // first.setSelectedSensorPosition(0);
        m_second = second;
        m_secondMinRange = Constants.ArmValues.SECOND_ARM_MIN;
        m_secondMaxRange = Constants.ArmValues.SECOND_ARM_MAX;
        second.configFactoryDefault();
        second.selectProfileSlot(0, 0);
        second.config_kP(0, Constants.PidValues.SECOND_ARM_KP);
        second.config_kI(0, Constants.PidValues.SECOND_ARM_KI);
        second.config_kD(0, Constants.PidValues.SECOND_ARM_KD);
        // second.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("First Angle", Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION));
        SmartDashboard.putNumber("Second Angle", Constants.Conversions.ticksToAngle(m_second.getSelectedSensorPosition(), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION));
        SmartDashboard.putNumber("motor output", m_first.getMotorOutputPercent());
    }

    public static Arm getInstance(){
        if(m_instance == null){
            m_instance = new Arm(new TalonFX(Constants.Ports.FIRST_ARM_PORT), new TalonFX(Constants.Ports.SECOND_ARM_PORT));
        }
        return m_instance;
    }

    public void turnFirstTo(double angle) {
          double m_firstAngle = Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
          double m_firstTarget = Constants.Conversions.angleToTicks(m_firstAngle + Constants.Conversions.closestAngle(m_firstAngle, angle), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
          if (m_firstAngle <= m_firstMaxRange && m_firstAngle >= m_firstMinRange){
            if (Math.abs(m_firstAngle - Math.abs(Constants.Conversions.modulo(Constants.Conversions.ticksToAngle(m_firstTarget, Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION), 360))) > m_firstMaxRange + Constants.ArmValues.LIMIT_TOLERANCE || Math.abs(m_firstAngle - Math.abs(Constants.Conversions.modulo(Constants.Conversions.ticksToAngle(m_firstTarget, Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION), 360))) < m_firstMinRange - Constants.ArmValues.LIMIT_TOLERANCE){
              m_first.set(TalonFXControlMode.Position, m_firstTarget);
            }
            else {
              m_first.set(TalonFXControlMode.Position, Constants.Conversions.angleToTicks(m_firstAngle, Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION));
            }
          }
    }

    public void turnSecondTo(double angle) {
          double m_secondAngle = Constants.Conversions.ticksToAngle(m_second.getSelectedSensorPosition(), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
          double m_secondTarget = Constants.Conversions.angleToTicks(m_secondAngle + Constants.Conversions.closestAngle(m_secondAngle, angle), Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
          if (m_secondAngle <= m_secondMaxRange && m_secondAngle >= m_secondMinRange){
            if (Math.abs(m_secondAngle - Math.abs(Constants.Conversions.modulo(Constants.Conversions.ticksToAngle(m_secondTarget, Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION), 360))) > m_secondMaxRange + Constants.ArmValues.LIMIT_TOLERANCE || Math.abs(m_secondAngle - Math.abs(Constants.Conversions.modulo(Constants.Conversions.ticksToAngle(m_secondTarget, Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION), 360))) < m_secondMinRange - Constants.ArmValues.LIMIT_TOLERANCE){
              m_second.set(TalonFXControlMode.Position, m_secondTarget);
            }
            else {
              m_second.set(TalonFXControlMode.Position, Constants.Conversions.angleToTicks(m_secondAngle, Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION));
            }
          }
    }

    public TalonFX getFirst(){
      return m_first;
    }

    public TalonFX getSecond(){
      return m_second;
    }

}

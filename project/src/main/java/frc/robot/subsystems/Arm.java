package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDS;
import frc.robot.Constants.PidOldValuesDontUse;

public class Arm extends SubsystemBase {

  private static TalonFX m_first, m_second;
  private static double m_firstMinRange, m_firstMaxRange, m_secondMinRange, m_secondMaxRange;
  private static Arm m_instance;

  public static enum ARMCONF {
    FORWARD, MID, BACK;
  }

  private static boolean m_coneIn = false;

  private static ARMCONF m_currentconf = ARMCONF.MID;

  public Arm(TalonFX first, TalonFX second) {
    m_first = first;
    m_firstMinRange = Constants.ArmValues.FIRST_ARM_MIN;
    m_firstMaxRange = Constants.ArmValues.FIRST_ARM_MAX;
    first.configFactoryDefault();
    first.selectProfileSlot(0, 0);
    first.config_kP(0, Constants.PidOldValuesDontUse.FIRST_ARM_KP);
    first.config_kI(0, Constants.PidOldValuesDontUse.FIRST_ARM_KI);
    first.config_kD(0, Constants.PidOldValuesDontUse.FIRST_ARM_KD);
    first.setSensorPhase(true);
    // first.setSelectedSensorPosition(0);
    m_second = second;
    m_secondMinRange = Constants.ArmValues.SECOND_ARM_MIN;
    m_secondMaxRange = Constants.ArmValues.SECOND_ARM_MAX;
    second.configFactoryDefault();
    second.selectProfileSlot(0, 0);
    second.config_kP(0, Constants.PidOldValuesDontUse.SECOND_ARM_KP);
    second.config_kI(0, Constants.PidOldValuesDontUse.SECOND_ARM_KI);
    second.config_kD(0, Constants.PidOldValuesDontUse.SECOND_ARM_KD);
    m_coneIn = false;
    // kf in specific commands
    // second.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    double firstAng = Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(),
        Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
    SmartDashboard.putNumber("First Angle", firstAng);
    SmartDashboard.putNumber("Second Angle", Constants.Conversions.ticksToAngle(m_second.getSelectedSensorPosition(),
        Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION));
    SmartDashboard.putNumber("motor1 output", m_first.getMotorOutputPercent());
    SmartDashboard.putNumber("motor2 output", m_second.getMotorOutputPercent());
    SmartDashboard.putBoolean("coneInside", m_coneIn);
    if (firstAng < 15 && firstAng > -15) {
      m_currentconf = ARMCONF.MID;
    } else if (firstAng < 15) {
      m_currentconf = ARMCONF.FORWARD;
    } else {
      m_currentconf = ARMCONF.BACK;
    }
  }

  public static ARMCONF getConf() {
    return m_currentconf;
  }

  public static double getFirstAngle() {
    return Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(),
        Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
  }

  public static double getSecondAngle() {
    return Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(),
        Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
  }

  public static Arm getInstance() {
    if (m_instance == null) {
      m_instance = new Arm(new TalonFX(Constants.Ports.FIRST_ARM_PORT), new TalonFX(Constants.Ports.SECOND_ARM_PORT));
    }
    return m_instance;
  }

  public static void turnFirstTo(double angle) {
    double m_firstAngle = Constants.Conversions.ticksToAngle(m_first.getSelectedSensorPosition(),
        Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
    double m_firstTarget = Constants.Conversions.angleToTicks(
        m_firstAngle + Constants.Conversions.closestAngle(m_firstAngle, angle),
        Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
    if (m_firstAngle <= m_firstMaxRange && m_firstAngle >= m_firstMinRange) {
      if (Math
          .abs(m_firstAngle - Math.abs(Constants.Conversions.modulo(
              Constants.Conversions.ticksToAngle(m_firstTarget, Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION),
              360))) > m_firstMaxRange + Constants.ArmValues.LIMIT_TOLERANCE
          || Math.abs(m_firstAngle - Math.abs(Constants.Conversions.modulo(
              Constants.Conversions.ticksToAngle(m_firstTarget, Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION),
              360))) < m_firstMinRange - Constants.ArmValues.LIMIT_TOLERANCE) {
        m_first.set(TalonFXControlMode.Position, m_firstTarget);
      } else {
        m_first.set(TalonFXControlMode.Position,
            Constants.Conversions.angleToTicks(m_firstAngle, Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION));
      }
    }
  }
  /**
   * @deprecated use specific controller instead
   * @param kf1
   * @param kp1
   * @param ki1
   * @param kd1
   */
  public static void _setSecondFPID(double kf1, double kp1, double ki1, double kd1) {
    m_second.config_kF(0, kf1);
    m_second.config_kP(0, kp1);
    m_second.config_kI(0, ki1);
    m_second.config_kD(0, kd1);
  }

  /**
   * 
   * @return the current [f,p,i,d] parqameters depends of whther a cone is inside
   *         or not
   */
  public static double[] getCurrentFirstFPID() {
    if (m_coneIn) {
      double[] res = { PIDS.firstKfCone, PIDS.firstKpCone, PIDS.firstKiCone, PIDS.firstKdCone };
      return res;
    }
    double[] res = { PIDS.firstKf, PIDS.firstKp, PIDS.firstKi, PIDS.firstKd };
    return res;
  }

  /**
   * 
   * @return the current [f,p,i,d] parqameters depends of whther a cone is inside
   *         or not
   */
  public static double[] getCurrentSecondFPID() {
    if (m_coneIn) {
      double[] res = { PIDS.secondKfCone, PIDS.secondKpCone, PIDS.secondKiCone, PIDS.secondKdCone };
      return res;
    }
    double[] res = { PIDS.secondKf, PIDS.secondKp, PIDS.secondKi, PIDS.secondKd };
    return res;
  }

  // TODO: currently constants, probably should be depand on cos(ang) or somthing
  /**
   * 
   * @return the current firstStall
   */
  public static double getFirstStall() {
    if (m_coneIn) {
      return PIDS.firstStallCone;
    }
    return PIDS.firstStall;
  }

  /**
   * 
   * @return the current secondStall
   */
  public static double getSecondStall() {
    if (m_coneIn) {
      return PIDS.secondStallCone;
    }
    return PIDS.secondStall;
  }
  public static void setconeIn(boolean coneIn){
    m_coneIn = coneIn;
  }
  public static void toggleConeIn(){
    setconeIn(!m_coneIn);
  }
  public static boolean getconeIn(){
    return m_coneIn;
  }

  public static void turnSecondTo(double angle) {
    double m_secondAngle = Constants.Conversions.ticksToAngle(m_second.getSelectedSensorPosition(),
        Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
    double m_secondTarget = Constants.Conversions.angleToTicks(
        m_secondAngle + Constants.Conversions.closestAngle(m_secondAngle, angle),
        Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
    if (m_secondAngle <= m_secondMaxRange && m_secondAngle >= m_secondMinRange) {
      if (Math
          .abs(m_secondAngle - Math.abs(Constants.Conversions.modulo(
              Constants.Conversions.ticksToAngle(m_secondTarget, Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION),
              360))) > m_secondMaxRange + Constants.ArmValues.LIMIT_TOLERANCE
          || Math.abs(m_secondAngle - Math.abs(Constants.Conversions.modulo(
              Constants.Conversions.ticksToAngle(m_secondTarget, Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION),
              360))) < m_secondMinRange - Constants.ArmValues.LIMIT_TOLERANCE) {
        m_second.set(TalonFXControlMode.Position, m_secondTarget);
      } else {
        m_second.set(TalonFXControlMode.Position,
            Constants.Conversions.angleToTicks(m_secondAngle, Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION));
      }
    }
  }

  public static TalonFX getFirst() {
    return m_first;
  }

  public static TalonFX getSecond() {
    return m_second;
  }

}

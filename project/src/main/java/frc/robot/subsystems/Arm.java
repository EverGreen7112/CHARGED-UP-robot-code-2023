package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDS;

public class Arm extends SubsystemBase {

  private static CANSparkMax m_first, m_second;
  private static SparkMaxPIDController m_firstPIDController, m_secondPIDController;
  private static double m_firstMinRange, m_firstMaxRange, m_secondMinRange, m_secondMaxRange;
  private static Arm m_instance;

  // privates because shoul be get only with the corresponding functions
  private final static double firstKf = 0.2;
  private final static double firstKp = 0;
  private final static double firstKi = 0;
  private final static double firstKd = 0;
  private final static double firstStall = 0.05;

  private final static double firstKfCone = firstKf * 1.5;
  private final static double firstKpCone = firstKp * 1.3;
  private final static double firstKiCone = firstKi * 1.3;
  private final static double firstKdCone = firstKd * 1.3;
  private final static double firstStallCone = firstKfCone;

  private final static double secondKf = 0;
  private final static double secondKp = 0;
  private final static double secondKi = 0;
  private final static double secondKd = 0;
  private final static double secondStall = 0;

  private final static double secondKfCone = firstKf * 1.5;
  private final static double secondKpCone = firstKp * 1.3;
  private final static double secondKiCone = firstKi * 1.3;
  private final static double secondKdCone = firstKd * 1.3;
  private final static double secondStallCone = firstStall * 1.5;

  public final static double firstArmTol = 1;
  public final static double firstArmVTol = 20;

  public static enum ARMCONF {
    FORWARD, MID, BACK;
  }

  private static boolean m_coneIn = false;

  private static ARMCONF m_currentconf = ARMCONF.MID;

  private Arm(CANSparkMax first, CANSparkMax second) {
    m_first = first;
    m_firstMinRange = Constants.ArmValues.FIRST_ARM_MIN;
    m_firstMaxRange = Constants.ArmValues.FIRST_ARM_MAX;
    m_first.restoreFactoryDefaults();
    m_first.setIdleMode(IdleMode.kBrake);
    m_firstPIDController = m_first.getPIDController();
    m_firstPIDController.setP(Constants.PidOldValuesDontUse.FIRST_ARM_KP);
    m_firstPIDController.setI(Constants.PidOldValuesDontUse.FIRST_ARM_KI);
    m_firstPIDController.setD(Constants.PidOldValuesDontUse.FIRST_ARM_KD);
    m_first.getEncoder().setPositionConversionFactor(4096);

    // first.setSelectedSensorPosition(0);
    m_second = second;
    m_secondMinRange = Constants.ArmValues.SECOND_ARM_MIN;
    m_secondMaxRange = Constants.ArmValues.SECOND_ARM_MAX;
    m_second.restoreFactoryDefaults();
    m_second.setIdleMode(IdleMode.kBrake);
    m_secondPIDController = m_second.getPIDController();
    m_secondPIDController.setP(Constants.PidOldValuesDontUse.SECOND_ARM_KP);
    m_secondPIDController.setI(Constants.PidOldValuesDontUse.SECOND_ARM_KI);
    m_secondPIDController.setD(Constants.PidOldValuesDontUse.SECOND_ARM_KD);
    m_second.getEncoder().setPositionConversionFactor(4096);
    m_coneIn = false;
    // kf in specific commands
    // second.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    double firstAng = Constants.Conversions.ticksToAngle(m_first.getEncoder().getPosition(),
        Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
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
    return Constants.Conversions.ticksToAngle(m_first.getEncoder().getPosition(),  
       Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION);
      //  Constants.Conversions.ticksToAngle(Arm.getFirst().getEncoder().getPosition(), Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION)
  }

  public static double getSecondAngle() {
    return Constants.Conversions.ticksToAngle(m_second.getEncoder().getPosition(),
        Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION);
  }

  public static Arm getInstance() {
    if (m_instance == null) {
      m_instance = new Arm(new CANSparkMax(Constants.Ports.FIRST_ARM_PORT, MotorType.kBrushless),
          new CANSparkMax(Constants.Ports.SECOND_ARM_PORT, MotorType.kBrushless));
    }
    return m_instance;
  }

  public static void turnFirstTo(double angle) {
    double m_firstAngle = Constants.Conversions.ticksToAngle(m_first.getEncoder().getPosition(),
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
        m_firstPIDController.setReference(m_firstTarget, ControlType.kPosition);
      } else {
        m_firstPIDController.setReference(
            Constants.Conversions.angleToTicks(m_firstAngle, Constants.Values.FIRST_ARM_TICKS_PER_REVOLUTION),
            ControlType.kPosition);
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
    m_secondPIDController.setFF(kf1);
    m_secondPIDController.setP(kp1);
    m_secondPIDController.setI(ki1);
    m_secondPIDController.setD(kd1);
  }

  /**
   * 
   * @return the current [f,p,i,d] parqameters depends of whther a cone is inside
   *         or not
   */
  public static double[] getFirstFPID() {
    if (m_coneIn) {
      double[] res = { firstKfCone, firstKpCone, firstKiCone, firstKdCone };
      return res;
    }
    double[] res = { firstKf, firstKp, firstKi, firstKd };
    return res;
  }

  /**
   * 
   * @return the current [f,p,i,d] parqameters depends of whther a cone is inside
   *         or not
   */
  public static double[] getSecondUpFPID() {
    if (m_coneIn) {
      double[] res = { secondKfCone, secondKpCone, secondKiCone, secondKdCone };
      return res;
    }
    double[] res = { secondKf, secondKp, secondKi, secondKd };
    return res;
  }

  // TODO: currently constants, probably should be depand on cos(ang) or somthing
  /**
   * 
   * @return the current firstStall
   */
  public static double getFirstStall() {
    // if (m_coneIn) {
    //   return firstStallCone;
    // }
    // return firstStall;
   if(Arm.getFirstAngle()>0){
     return firstStall*0.8;
   }else{
    return firstStall*-1;
   }
  }

  /**
   * 
   * @return the current secondStall
   */
  public static double getSecondStall() {
    if (m_coneIn) {
      return secondStallCone;
    }
    return secondStall;
  }

  public static void setconeIn(boolean coneIn) {
    m_coneIn = coneIn;
  }

  public static void toggleConeIn() {
    setconeIn(!m_coneIn);
  }

  public static boolean getconeIn() {
    return m_coneIn;
  }

  public static void turnSecondTo(double angle) {
    double m_secondAngle = Constants.Conversions.ticksToAngle(m_second.getEncoder().getPosition(),
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
        m_secondPIDController.setReference(m_secondTarget, ControlType.kPosition);
      } else {
        m_secondPIDController.setReference(
            Constants.Conversions.angleToTicks(m_secondAngle, Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION),
            ControlType.kPosition);
      }
    }
  }

  public static CANSparkMax getFirst() {
    return m_first;
  }

  public static CANSparkMax getSecond() {
    return m_second;
  }
  public static void stall1(){
    m_first.set(getFirstStall());
  }
  public static void stall2(){
    m_first.set(getSecondStall());
  }
  // }
}

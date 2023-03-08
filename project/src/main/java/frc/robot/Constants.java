// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final class Ports{
    //tank drive chassis
    public static final int 
    LEFT_FRONT_PORT = 2,
    LEFT_MIDDLE_PORT = 14,
    LEFT_BACK_PORT = 10,
    RIGHT_FRONT_PORT = 15,
    RIGHT_MIDDLE_PORT = 12,
    RIGHT_BACK_PORT = 11,
    //swerve constants 
    FLY_WHEEL_PORT = 0,
    RIGHT_UP_DRIVE_MOTOR_PORT = 0,
    RIGHT_UP_TURN_MOTOR_PORT = 0,
    LEFT_UP_DRIVE_MOTOR_PORT = 0,
    LEFT_UP_TURN_MOTOR_PORT = 0,
    RIGHT_DOWN_DRIVE_MOTOR_PORT = 0,
    RIGHT_DOWN_TURN_MOTOR_PORT = 0,
    LEFT_DOWN_DRIVE_MOTOR_PORT = 0,
    LEFT_DOWN_TURN_MOTOR_PORT = 0,
    //arm ports
    FIRST_ARM_PORT = 13,
    SECOND_ARM_PORT = 3,
    GRIPPER_PORT = 1,
    //limit switch ports
    CLOSED_LIMIT_SWITCH = 7,
    OPENED_LIMIT_SWITCH = 9,
    CUBE_LIMIT_SWITCH = 8,


    // vision ports
    REFLECTOR_PORT = 5802,
    RECIEVE_LOCATION_PORT = 5800
    ;
  }


  public static class ArmValues {

    public static final double 
            FIRST_ARM_LENGTH = 0.9 //IN METERS
            ,SECOND_ARM_LENGTH = 0.45
            ,FIRST_ARM_MIN = 0
            ,FIRST_ARM_MAX = 360,
            FIRST_ARM_R_MAX = 127,
            FIRST_ARM_L_MIN =-130
            ,SECOND_ARM_MIN = 0
            ,SECOND_ARM_MAX = 180
            ,JOYSTICK_TOLERANCE = 0.5
            ,LIMIT_TOLERANCE = 20
            //angles for the arms
            ,FIRST_PICKUP_TOP_ANGLE = 120
            ,SECOND_PICKUP_TOP_ANGLE = 10
            ,FIRST_PICKUP_DOWN_ANGLE = 0
            ,SECOND_PICKUP_DONW_ANGLE = 0
            ;
  }
  
  public final static  class Speeds{
    public final static Supplier<Double> constantSpeed = ()->SmartDashboard.getNumber("changeDriveSpeed",0.7);
    public final static double constantPropConst = 0.07;
    // public final static Supplier<Double> driveMax = () -> {
    //   return SmartDashboard.getNumber("robot speed", 0.2);
    // };
    public static final double GRIPPER_OPEN_SPEED = 0.15;
    public static final double GRIPPER_CLOSE_SPEED = -0.25;
    public static final double GRIPPER_DEFUALT_CLOSE = -0.15;

    public static final double GRIPPER_CLOSE_CUBE = -0.14;

    public static final double TURBO = 1.5;
    public static final double SLOW = 0.5;
  }

  public static class JoystickPorts {
    public static final int rightJoystick = 0,
        leftJoystick = 1,
        operator = 2;
  }
//custom button ports according to the operator
  public static class ButtonPorts {
    public static final int
        X = 1,
        A = 2,
        B = 3,
        Y = 4,
        LB = 5,
        RB = 6,
        LT = 7,
        RT = 8,
        BACK = 9,
        START = 10,
        LEFT_JOYSTICK = 11,
        RIGHT_JOYSTICK = 12,
        OPERATOR_LEFT_JOYSTICK_X = 0,
        OPERATOR_LEFT_JOYSTICK_Y = 1,
        OPERATOR_RIGHT_JOYSTICK_X = 2,
        OPERATOR_RIGHT_JOYSTICK_Y = 3;
  }

  public static class MotorPorts {
    public static final int FLY_WHEEL_PORT = 0, FIRST_ARM_PORT = 0, SECOND_ARM_PORT = 0, GRIPPER_PORT = 0;
  }

  public static class PIDS {

    public final static double driveKp = 0.05;
    public final static double driveKi = 0.009;
    public final static double driveKd = 0.007;
    public final static double drivePTolerance = 0.02;
    public final static double driveVTolerance = 1;

    public final static double rotateKp = 0.05 / 40;
    public final static double rotateKi = 0.001 / 20;
    public final static double rotateKd = 0.003 / 20;
    public final static double rotatePTolerance = 2;
    public final static double rotateVTolerance = 10;

    public final static double velKp = 0.05 / 75;
    public final static double velKi = 0.007 / 75;
    public final static double velKd = 0.007 / 75;
    public final static double velPTolerance = 2;
    public final static double velVTolerance = 10;

    public final static double firstKf = 0;
    public final static double firstKp = 0;
    public final static double firstKi = 0;
    public final static double firstKd = 0;
    public final static double firstStall = firstKf;

    public final static double firstKfCone = firstKf*1.5;
    public final static double firstKpCone = firstKp*1.3;
    public final static double firstKiCone = firstKi*1.3;
    public final static double firstKdCone= firstKd *1.3;
    public final static double firstStallCone = firstKfCone;

    public final static double secondKf = 0;
    public final static double secondKp = 0;
    public final static double secondKi = 0;
    public final static double secondKd = 0;
    public final static double secondStall = 0;

    public final static double secondKfCone = firstKf*1.5;
    public final static double secondKpCone = firstKp*1.3;
    public final static double secondKiCone = firstKi*1.3;
    public final static double secondKdCone= firstKd *1.3;
    public final static double secondStallCone = firstStall*1.5;





  }

public final class Values {
  public static final double 
      FIRST_ARM_TICKS_PER_REVOLUTION = 2048*16.9142857,
      SECOND_ARM_TICKS_PER_REVOLUTION = 2048*10.8;
  public static final double 
  TANKDRIVE_WHEEL_RADIUS = 0.076,
  DISTANCE_BETWEEN_LEFT_TO_RIGHT = 0.5; //in meters;
  //swerve constants
  public static final double
  WHEEL_POSITION_PID_TOLERANCE = 0.5
  ,WHEEL_POSITION_PID_KP = 0.01 * 9
  ,WHEEL_POSITION_PID_KI = 0.00000
  ,WHEEL_POSITION_PID_KD = 0.000001
  ,TURN_SWERVE_WHEEL_PID_KP = 0.02
  ,TURN_SWERVE_WHEEL_PID_KI = 0
  ,TURN_SWERVE_WHEEL_PID_KD = 0
  ,DRIVE_SWERVE_WHEEL_PID_KP = 0.02
  ,DRIVE_SWERVE_WHEEL_PID_KI = 0
  ,DRIVE_SWERVE_WHEEL_PID_KD = 0
  ,SWERVE_WHEEL_RADIUS = 0
  ,SWERVE_LENGTH = 0
  ,SWERVE_WIDTH = 0
  ,MAX_SWERVE_SPEED = 3 //in m/s
  ,DISTANCE_PER_TICK= 2 * Math.PI *TANKDRIVE_WHEEL_RADIUS/10.97;  // TODO: check
  ;
  
}

public static class PidOldValuesDontUse {
  public static double 
          FIRST_ARM_KP = 0.07,
          FIRST_ARM_KI = 0.000006
          ,FIRST_ARM_KD = 0
          ,SECOND_ARM_KP = 0.02
          ,SECOND_ARM_KI = 0
          ,SECOND_ARM_KD = 0
          ,SECOND_ARM_KF = 0.02,
          SECOND_ARM_STALL_SPEED = 0.06,
          _SECOND_ARM_KP = 0.01
          ,_SECOND_ARM_KI = 0
          ,_SECOND_ARM_KD = 0
          ,_SECOND_ARM_KF = 0.02,
          _SECOND_ARM_STALL_SPEED = 0.06,

          //anti
          FIRST_ARM_ANTI_KP = 0.001,
          FIRST_ARM_ANTI_KF = 0.2,
          //bacl
          FIRST_ARM_BACK_KP = 0.1,
          FIRST_ARM_BACK_KI = 0.00000,
          FIRST_ARM_BACK_KD = 0.000;
         
}

public final static class Conversions {
  public static double rpm2ms(double wheelRadius, double rpm){
    double rps = rpm / 60;
    return wheelRadius * 2 * Math.PI * rps;
  }
    //TPR is ticks per revolution
    public static double angleToTicks(double angle, double TPR){
        return TPR / ((double) 360 / angle);
    }
    //TPR is ticks per revolution
    public static double ticksToAngle(double ticks, double TPR){
        return (ticks * 360.0)/ TPR;
    }

    public static double ticksToMeters(double ticks, double TPR) {
      return Math.PI * 0.1524 * (ticks / TPR); // 0.1524 is the wheel diameter in meters
    }
    
    public static double closestAngle(double a, double b) {
      // get direction
      double dir = modulo(b, 360.0) - modulo(a, 360.0);

      // convert from -360 to 360 to -180 to 180
      if (Math.abs(dir) > 180.0) {
          dir = -(Math.signum(dir) * 360.0) + dir;
      }
      return dir;
  }
  public static Vector2D rotateZ(Vector2D vector,double angle) { // angle in radians

    //normalize(vector); // No  need to normalize, vector is already ok...
  
    float x1 = (float)(vector.x * Math.cos(angle) - vector.y * Math.sin(angle));
  
    float y1 = (float)(vector.x * Math.sin(angle) + vector.y * Math.cos(angle)) ;
  
    return new Vector2D(x1, y1);
  
  }


  public static double modulo(double a, double b) {
      return ((a % b) + b) % b;
  }
}

}

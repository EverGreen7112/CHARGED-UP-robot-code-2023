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

  public final class Ports {
    // tank drive chassis
    public static final int LEFT_FRONT_PORT = 2,
        LEFT_MIDDLE_PORT = 14,
        LEFT_BACK_PORT = 10,
        RIGHT_FRONT_PORT = 15,
        RIGHT_MIDDLE_PORT = 12,
        RIGHT_BACK_PORT = 11,
        // swerve constants
        FLY_WHEEL_PORT = 0,
        RIGHT_UP_DRIVE_MOTOR_PORT = 0,
        RIGHT_UP_TURN_MOTOR_PORT = 0,
        LEFT_UP_DRIVE_MOTOR_PORT = 0,
        LEFT_UP_TURN_MOTOR_PORT = 0,
        RIGHT_DOWN_DRIVE_MOTOR_PORT = 0,
        RIGHT_DOWN_TURN_MOTOR_PORT = 0,
        LEFT_DOWN_DRIVE_MOTOR_PORT = 0,
        LEFT_DOWN_TURN_MOTOR_PORT = 0;
  }

  public final static class Speeds {
    public final static double constantSpeed = 0.4;
    public final static double constantPropConst = 0.07;
    public final static Supplier<Double> driveMax = () -> {
      return SmartDashboard.getNumber("robot speed", 0.4);
    };
    public static final double GRIPPER_SPEED = 0.1;
  }

  public static class ButtonPorts {
    public static final int GRAB = 0,
        OPEN_GRIPPER = 2,
        CLOSE_GRIPPER = 3;
  }

  public static class JoystickPorts {
    public static final int rightJoystick = 0,
        leftJoystick = 1,
        operator = 2;
  }

  public static class Conversions {
    public static double rpm2ms(double wheelRadius, double rpm) {
      double rps = rpm / 60;
      return wheelRadius * 2 * Math.PI * rps;
    }

    public static double angleToTicks(double angle) {
      return Constants.Values.TICKS_PER_REVOLUTIONS / ((double) 360 / angle);

    }

  

    public static double ticksToAngle(double ticks) {
      return (ticks * 360.0) / Constants.Values.TICKS_PER_REVOLUTIONS;
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

    public static double modulo(double a, double b) {
      return ((a % b) + b) % b;
    }
    public static Vector2D rotateZ(Vector2D vector,double angle) { // angle in radians

      //normalize(vector); // No  need to normalize, vector is already ok...
    
      float x1 = (float)(vector.x * Math.cos(angle) - vector.y * Math.sin(angle));
    
      float y1 = (float)(vector.x * Math.sin(angle) + vector.y * Math.cos(angle)) ;
    
      return new Vector2D(x1, y1);
    
    }
  }

  public static class MotorPorts {
    public static final int FLY_WHEEL_PORT = 0, FIRST_ARM_PORT = 0, SECOND_ARM_PORT = 0, GRIPPER_PORT = 0;
  }

  public static class Values {
    public static final int TICKS_PER_REVOLUTIONS = 8196;
    public static double TANKDRIVE_WHEEL_RADIUS = 0.76,
        DISTANCE_BETWEEN_LEFT_TO_RIGHT = 0.5; // in meters;
    // swerve constants
    public static final double WHEEL_POSITION_PID_TOLERANCE = 0.5, WHEEL_POSITION_PID_KP = 0.01 * 9,
        WHEEL_POSITION_PID_KI = 0.00000, WHEEL_POSITION_PID_KD = 0.000001, TURN_SWERVE_WHEEL_PID_KP = 0.02,
        TURN_SWERVE_WHEEL_PID_KI = 0, TURN_SWERVE_WHEEL_PID_KD = 0, DRIVE_SWERVE_WHEEL_PID_KP = 0.02,
        DRIVE_SWERVE_WHEEL_PID_KI = 0, DRIVE_SWERVE_WHEEL_PID_KD = 0, SWERVE_WHEEL_RADIUS = 0, SWERVE_LENGTH = 0,
        SWERVE_WIDTH = 0, MAX_SWERVE_SPEED = 3,
        DISTANCE_PER_THICK= 2 * Math.PI *TANKDRIVE_WHEEL_RADIUS/10.97;  // TODO: check
    ;
  }

  public static class PIDS {
    public static final double FIRST_ARM_KP = 0.0000001, FIRST_ARM_KI = 0, FIRST_ARM_KD = 0.000001,
        SECOND_ARM_KP = 0.00000001, SECOND_ARM_KI = 0, SECOND_ARM_KD = 0;
    public final static double driveKp = 0.05;
    public final static double driveKi = 0.007;
    public final static double driveKd = 0.007;
    public final static double drivePTolerance = 0.02;
    public final static double driveVTolerance = 1;

    public final static double rotateKp = 0.05 / 75;
    public final static double rotateKi = 0.007 / 75;
    public final static double rotateKd = 0.007 / 75;
    public final static double rotatePTolerance = 2;
    public final static double rotateVTolerance = 10;

    public final static double velKp = 0.05 / 75;
    public final static double velKi = 0.007 / 75;
    public final static double velKd = 0.007 / 75;
    public final static double velPTolerance = 2;
    public final static double velVTolerance = 10;
  }

}

package frc.robot;

public class Utillities {

    public static boolean joysticksOutOfRange(){
        return joystick1OutOfRange() || joystic2OutOfRange();
    }
    public static boolean joysticksStrengthOutOfRange(){
        return joystick1OutOfRangeStrength() || joystick2OutOfRangeStrength();
    }

    // operator left joystick for angle
    public static boolean joystick1OutOfRange() {
        return Math.abs(RobotContainer.m_operator.getRawAxis(Constants.ButtonPorts.OPERATOR_LEFT_JOYSTICK_X)) < Constants.ArmValues.JOYSTICK_ANGLE_TOLERANCE && Math.abs(RobotContainer.m_operator.getRawAxis(Constants.ButtonPorts.OPERATOR_LEFT_JOYSTICK_Y)) < Constants.ArmValues.JOYSTICK_ANGLE_TOLERANCE;
    }

    // operator right joystick for angle
    public static boolean joystic2OutOfRange() {
        return Math.abs(RobotContainer.m_operator.getRawAxis(Constants.ButtonPorts.OPERATOR_RIGHT_JOYSTICK_X)) < Constants.ArmValues.JOYSTICK_ANGLE_TOLERANCE && Math.abs(RobotContainer.m_operator.getRawAxis(Constants.ButtonPorts.OPERATOR_RIGHT_JOYSTICK_Y)) < Constants.ArmValues.JOYSTICK_ANGLE_TOLERANCE;
    }

    // operator left joystick for strength
    public static boolean joystick1OutOfRangeStrength() {
        return Math.abs(RobotContainer.m_operator.getRawAxis(Constants.ButtonPorts.OPERATOR_LEFT_JOYSTICK_Y)) < Constants.ArmValues.JOYSTICK_STRENGTH_TOLERANCE;
    }

    // operator right joystick for strength
    public static boolean joystick2OutOfRangeStrength() {
        return Math.abs(RobotContainer.m_operator.getRawAxis(Constants.ButtonPorts.OPERATOR_RIGHT_JOYSTICK_Y)) < Constants.ArmValues.JOYSTICK_STRENGTH_TOLERANCE;
    }

    // operator right joystick angle
    public static double getRightJoystickAngle() {
        return Math.toDegrees(Math.atan2(RobotContainer.m_operator.getRawAxis(Constants.ButtonPorts.OPERATOR_RIGHT_JOYSTICK_X), RobotContainer.m_operator.getRawAxis(Constants.ButtonPorts.OPERATOR_RIGHT_JOYSTICK_Y)));
    }

    // operator left joystick angle
    public static double getLeftJoystickAngle() {
        return Math.toDegrees(Math.atan2(RobotContainer.m_operator.getRawAxis(Constants.ButtonPorts.OPERATOR_LEFT_JOYSTICK_X), RobotContainer.m_operator.getRawAxis(Constants.ButtonPorts.OPERATOR_LEFT_JOYSTICK_Y)));
    }
}

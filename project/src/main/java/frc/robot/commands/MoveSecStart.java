package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.PidValues;
import frc.robot.subsystems.Arm;

public class MoveSecStart extends CommandBase {
    private TalonFX m_first;
    private TalonFX m_second;
    private double m_firstArmAngle;
    private double m_secondArmAngle;
    private double startDir = 1;
    private boolean cross =false;
    public MoveSecStart() {
        addRequirements(Arm.getInstance());
        m_first = Arm.getInstance().getFirst();
        m_second = Arm.getInstance().getSecond();
    }
    
    @Override
    public void initialize() {
        // m_first.config_kP(0, Constants.PidValues.FIRST_ARM_KP * 1.6);
        // m_first.config_kF(0, 0.35);
        m_second.config_kP(0, 0.02);
        m_second.config_kF(0,-1.3*Math.signum(Arm.getInstance().getFirstAngle()));
        m_second.config_kD(0, 0.0003);
        m_second.config_kI(0, 0);
        addRequirements(Arm.getInstance());
    }
   
    public boolean isKindaFinished(){
        return m_secondArmAngle < Constants.ArmValues.LIMIT_TOLERANCE + 0
        && m_secondArmAngle > 0 - Constants.ArmValues.LIMIT_TOLERANCE;
    }
    @Override
    public void execute() {
        m_second.set(TalonFXControlMode.Position,
        Constants.Conversions.angleToTicks(-1,
        Constants.Values.SECOND_ARM_TICKS_PER_REVOLUTION));
        m_second.config_kF(0,-1.3*Math.signum(Arm.getInstance().getFirstAngle()));
    }

}

package frc.robot.commands.ChassisPid;


import java.util.function.DoubleSupplier;

import frc.robot.Constants.PIDS;
import frc.robot.subsystems.Chassis;

public class ChasisSetPointPosPID extends ChassisPostionPID {

    public ChasisSetPointPosPID(DoubleSupplier setpointSource) {
        super(setpointSource);
    }

    @Override
    public boolean isFinished() {
             return (Math.abs(getCurDistance() - m_setPoint.getAsDouble()) < PIDS.drivePTolerance
                && (Math.abs(Chassis.getVelocity()) < PIDS.driveVTolerance));
    }

}

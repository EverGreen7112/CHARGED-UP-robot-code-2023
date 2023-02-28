package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PowerBySupplier extends CommandBase {
    Supplier<Double> m_sup;
    public PowerBySupplier(Supplier<Double> doubSup) {
        m_sup = doubSup;
    }
    
    
}

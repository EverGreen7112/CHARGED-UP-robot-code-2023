package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.General.Commands;
import frc.robot.subsystems.Chassis;

public class DriveToGamePiece extends CommandBase{

    @Override
    public void initialize(){
        Commands.placeGamePieceAuto.schedule();;
    }
    
}

package frc.robot.Macros;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LeverSubsystem;
import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.LeverAutoCmd;


public class LeverMac extends SequentialCommandGroup {

    public LeverMac(LeverSubsystem leverSubsystem, LauncherSubsystem launcherSubsystem){
        // 5. Add commands to the SequentialCommandGroup
        addCommands(

        new LeverAutoCmd(leverSubsystem),
        new InstantCommand(() -> launcherSubsystem.front_Rollers(-0.1)),
        
        new WaitCommand(0.5),
        
        new InstantCommand(() -> launcherSubsystem.stopRollers()),
        new WaitCommand(1),

        new InstantCommand(() -> launcherSubsystem.all_Rollers(LauncherConstants.kRollers_Speed1)),
        new WaitCommand(0.2),

        new InstantCommand(() -> launcherSubsystem.stopRollers())
        
        );
    }
}

package frc.robot.Macros;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LauncherSubsystem;


public class SlowShooterMac extends SequentialCommandGroup {

    public SlowShooterMac(LauncherSubsystem launcherSubsystem, double speed){
        // 5. Add commands to the SequentialCommandGroup
        addCommands(

        new InstantCommand(() -> launcherSubsystem.all_Rollers(speed)),
        new WaitCommand(0.5),
        
        new InstantCommand(() -> launcherSubsystem.stopRollers())
        
        );
    }
}

package frc.robot.Macros;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.Constants.IntakeConstants;


public class LoadToAmp extends SequentialCommandGroup {

    public LoadToAmp(IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem){
        // 5. Add commands to the SequentialCommandGroup
        addCommands(

        new InstantCommand(() -> intakeSubsystem.setLoader(IntakeConstants.kLoaderSpeed)),
        new InstantCommand(() -> launcherSubsystem.back_Rollers(0.15)),
        
        new WaitCommand(1),
        
        new InstantCommand(() -> intakeSubsystem.stopIntake()),
        new InstantCommand(() -> launcherSubsystem.stopRollers())
        
        );
    }
}

package frc.robot.Macros;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;


public class ShooterForAutoMac extends SequentialCommandGroup {

    public ShooterForAutoMac(LauncherSubsystem launcherSubsystem, IntakeSubsystem intakeSubsystem){
        // 5. Add commands to the SequentialCommandGroup
        addCommands(

        new InstantCommand(() -> launcherSubsystem.all_Rollers(LauncherConstants.kRollers_Speed1)),
        new WaitCommand(0.8),

        new InstantCommand(() -> intakeSubsystem.setLoader(IntakeConstants.kLoaderSpeed)),
        new WaitCommand(1),
        
        new InstantCommand(() -> launcherSubsystem.stopRollers()),
        new InstantCommand(() -> intakeSubsystem.stopIntake())
        
        );
    }
}

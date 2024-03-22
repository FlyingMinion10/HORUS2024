package frc.robot.Macros;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;


public class IntakeMac extends SequentialCommandGroup {

    public IntakeMac(IntakeSubsystem intakeSubsystem){
        // 5. Add commands to the SequentialCommandGroup
        addCommands(

        new InstantCommand(() -> intakeSubsystem.setIntake(IntakeConstants.kIntakeSpeed)),
        new InstantCommand(() -> intakeSubsystem.setLoader(IntakeConstants.kLoaderSpeed)),
        
        new WaitCommand(1),
        
        new InstantCommand(() -> intakeSubsystem.stopIntake())
        
        );
    }
}

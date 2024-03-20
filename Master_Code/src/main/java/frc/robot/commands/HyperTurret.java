package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;


public class HyperTurret extends Command {
    private final LauncherSubsystem launcherSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;


    public HyperTurret(LauncherSubsystem launcherSubsystem_i, SwerveSubsystem swerveSubsystem_i, IntakeSubsystem intakeSubsystem_i) {
        this.launcherSubsystem = launcherSubsystem_i;
        this.swerveSubsystem = swerveSubsystem_i;
        this.intakeSubsystem = intakeSubsystem_i;
        addRequirements(launcherSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Launcher Cmd started!");
        swerveSubsystem.stopModules();
        
    }

    @Override
    public void execute() {
        launcherSubsystem.all_Rollers(LauncherConstants.kRollers_Speed1); // Revolucionar los rollers de arriba
        intakeSubsystem.setLoader(IntakeConstants.kLoaderSpeed);
        intakeSubsystem.setIntake(IntakeConstants.kIntakeSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        launcherSubsystem.stopRollers();
        intakeSubsystem.stopIntake();
        System.out.println("Launcher Cmd ended!");
    }

    @Override
    public boolean isFinished() {
        // El comando termina después de completar la última etapa
        return false;
    }
}

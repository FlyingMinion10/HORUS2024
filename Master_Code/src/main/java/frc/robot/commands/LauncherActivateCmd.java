package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;


public class LauncherActivateCmd extends Command {
    private final LauncherSubsystem launcherSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    //private TejuinoBoard leds = new TejuinoBoard();
    private final Timer timer = new Timer();
    private int stage = 0;

    public LauncherActivateCmd(LauncherSubsystem launcherSubsystem_i, SwerveSubsystem swerveSubsystem_i, IntakeSubsystem intakeSubsystem_i) {
        this.launcherSubsystem = launcherSubsystem_i;
        this.swerveSubsystem = swerveSubsystem_i;
        this.intakeSubsystem = intakeSubsystem_i;
        addRequirements(launcherSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Launcher Cmd started!");
        /*leds.init(1);
        leds.all_leds_purple(0);
        leds.all_leds_purple(1);*/
        swerveSubsystem.stopModules();
        stage = 0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        
        switch (stage) {
            case 0:
                launcherSubsystem.all_Rollers(LauncherConstants.kRollers_Speed1);// Revolucionar los rollers de arriba
                if (timer.hasElapsed(0.8)) {
                    intakeSubsystem.setLoader(IntakeConstants.kLoaderSpeed);
                    stage++;
                    timer.reset();
                }
                break;
            case 1:
                if (timer.hasElapsed(1)) {
                    launcherSubsystem.stopRollers();
                    stage++;
                }
                break;
        }

    }

    @Override
    public void end(boolean interrupted) {
        launcherSubsystem.stopRollers();
        intakeSubsystem.setLoader(0);
        System.out.println("Launcher Cmd ended!");
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // El comando termina después de completar la última etapa
        return stage > 1;
    }
}

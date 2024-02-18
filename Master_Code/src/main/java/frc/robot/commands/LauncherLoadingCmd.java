package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class LauncherLoadingCmd extends Command { // This Command is a once-pressed Type
    private final LauncherSubsystem launcherSubsystem;
    private final double loadingSpeed;

    public LauncherLoadingCmd(LauncherSubsystem launcherSubsystem_i, double loadingSpeed) {
        this.launcherSubsystem = launcherSubsystem_i;
        this.loadingSpeed = loadingSpeed;
        addRequirements(launcherSubsystem);
    }

    Timer timer = new Timer();


    @Override
    public void initialize() {
        System.out.println("LauncherCmd started!");
    }

    @Override
    public void execute() {

        launcherSubsystem.lower_Rollers(loadingSpeed); // Subir aro hasta perder traccion
        Timer.delay(1.3);
        launcherSubsystem.lower_Rollers(0);
        Timer.delay(0.5);

        launcherSubsystem.upper_Rollers(-loadingSpeed); // Bajar el aro para ajustarlo a punto de disparo
        launcherSubsystem.lower_Rollers(-loadingSpeed);
        Timer.delay(0.3);
        launcherSubsystem.stopRollers();
    }

    @Override
    public void end(boolean interrupted) {

        launcherSubsystem.stopRollers();
        System.out.println("LauncherCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;


public class LauncherFeederCmd extends Command { // This Command is a while-pressed Type
private final LauncherSubsystem launcherSubsystem;
    private final double feederSpeed;

    public LauncherFeederCmd(LauncherSubsystem launcherSubsystem_i, double feederSpeed) {
        this.launcherSubsystem = launcherSubsystem_i;
        this.feederSpeed = feederSpeed;
        addRequirements(launcherSubsystem);
    }


    @Override
    public void initialize() {
        System.out.println("Reload Cmd started!");
    }

    @Override
    public void execute() { // Los rollers giran hacia adentro para absorber el aro
        
        launcherSubsystem.upper_Rollers(-feederSpeed);
        launcherSubsystem.lower_Rollers(-feederSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        launcherSubsystem.stopRollers();
        System.out.println("Reload Cmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
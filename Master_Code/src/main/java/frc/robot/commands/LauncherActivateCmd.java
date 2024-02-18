package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.LauncherConstants;
import edu.wpi.first.wpilibj.Timer;


public class LauncherActivateCmd extends Command { // This Command is a once-pressed Type
    private final LauncherSubsystem launcherSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final double upperSpeed;

    //private final double lowerSpeed;

    public LauncherActivateCmd(LauncherSubsystem launcherSubsystem_i, SwerveSubsystem swerveSubsystem_i, double upperSpeed) {
        this.launcherSubsystem = launcherSubsystem_i;
        this.swerveSubsystem = swerveSubsystem_i;
        this.upperSpeed = upperSpeed;
        //this.lowerSpeed = lowerSpeed;
        addRequirements(launcherSubsystem);
    }


    @Override
    public void initialize() {
        System.out.println("LauncherCmd started!");
    }

    @Override
    public void execute() {
        
        swerveSubsystem.stopModules();
        launcherSubsystem.upper_Rollers(upperSpeed);
        Timer.delay(0.8);
        launcherSubsystem.lower_Rollers(LauncherConstants.kLowerRoller_Speed);
        Timer.delay(1);
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
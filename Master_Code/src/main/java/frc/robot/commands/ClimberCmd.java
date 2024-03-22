package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberCmd extends Command { // This Command is a while-pressed Type
    private final ClimberSubsystem ClimberSubsystem;
    private final double speed;
    //private TejuinoBoard leds = new TejuinoBoard();

    public ClimberCmd(ClimberSubsystem ClimberSubsystem_i, double speed_i) {
        this.ClimberSubsystem = ClimberSubsystem_i;
        this.speed = speed_i;
        addRequirements(ClimberSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Climber Cmd started!");
        //leds.turn_off_all_leds(0);
        //leds.turn_off_all_leds(1);
    }

    @Override
    public void execute() { // Los rollers giran hacia adentro para absorber el aro
        ClimberSubsystem.setClimberSpeed(speed);
    }

    @Override 
    public void end(boolean interrupted) {

        ClimberSubsystem.stopClimber();
        System.out.println("Climber Cmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeverSubsystem;


public class LeverManualCmd extends Command { 
    private final LeverSubsystem leverSubsystem;
    private final double speed;
    //private TejuinoBoard leds = new TejuinoBoard();

    public LeverManualCmd(LeverSubsystem leverSubsystem_i, double speed_i) {
        this.leverSubsystem = leverSubsystem_i;
        this.speed = speed_i;
        addRequirements(leverSubsystem_i);
    }

    @Override
    public void initialize() {
        System.out.println("Manual lever");
    }

    @Override
    public void execute() {
        leverSubsystem.setLeverSpeed(speed);
    }

    @Override 
    public void end(boolean interrupted) {
        leverSubsystem.stopLever();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
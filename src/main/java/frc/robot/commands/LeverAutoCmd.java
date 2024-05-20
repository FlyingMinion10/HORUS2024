package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LeverConstants;
import frc.robot.subsystems.LeverSubsystem;


public class LeverAutoCmd extends Command { 
    private final LeverSubsystem leverSubsystem;
    //private TejuinoBoard leds = new TejuinoBoard();
    private boolean isAtZero = true;
    private boolean finished;

    public LeverAutoCmd(LeverSubsystem leverSubsystem_i) {
        this.leverSubsystem = leverSubsystem_i;
        addRequirements(leverSubsystem_i);
    }

    @Override
    public void initialize() {
        System.out.println("Changing position");
        finished = false;
    }

    @Override
    public void execute() {
        if (isAtZero) { 
            //leds.all_leds_blue(0);
            //leds.all_leds_blue(1);
            leverSubsystem.setLeverSpeed(LeverConstants.kLeverSpeeed);
            if (leverSubsystem.getPos() >= 35.7) { //32.7
                leverSubsystem.stopLever();
                finished = true; }

        } else if (!isAtZero) { 
            //leds.all_leds_blue(0);
            //leds.all_leds_blue(1);
            leverSubsystem.setLeverSpeed(-LeverConstants.kLeverSpeeed);
            if (leverSubsystem.getPos() <= 0.6) {
                leverSubsystem.stopLever();
                finished = true; }
        }
    }

    @Override 
    public void end(boolean interrupted) {
        isAtZero = !isAtZero;


    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
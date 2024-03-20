package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LeverConstants;
import frc.robot.subsystems.LeverSubsystem;


public class LeverToPosCmd extends Command { 
    private final LeverSubsystem leverSubsystem;
    private final double desiredPos;
    private boolean isAtZero = true;
    private boolean finished;

    public LeverToPosCmd(LeverSubsystem leverSubsystem_i, double desiredPosition_i) {
        this.leverSubsystem = leverSubsystem_i;
        this.desiredPos = desiredPosition_i;
        addRequirements(leverSubsystem_i);
    }

    @Override
    public void initialize() {
        System.out.println("Changing position");
        finished = false;
    }

    @Override
    public void execute() {
        
        if (leverSubsystem.getPos() < desiredPos) { 
            leverSubsystem.setLeverSpeed(LeverConstants.kLeverSpeeed);
            if (leverSubsystem.getPos() >= desiredPos) {
                leverSubsystem.stopLever();
                finished = true; }

        } else if (leverSubsystem.getPos() > desiredPos) { 
            leverSubsystem.setLeverSpeed(-LeverConstants.kLeverSpeeed);
            if (leverSubsystem.getPos() <= desiredPos) {
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
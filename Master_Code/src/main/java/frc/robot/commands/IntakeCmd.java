package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeCmd extends Command { // This Command is a while-pressed Type
    private final IntakeSubsystem intakeSubsystem;
    //private TejuinoBoard leds = new TejuinoBoard();

    public IntakeCmd(IntakeSubsystem intakeSubsystem_i) {
        this.intakeSubsystem = intakeSubsystem_i;
        addRequirements(intakeSubsystem);
    }


    @Override
    public void initialize() {
        System.out.println("Reload Cmd started!");
        //leds.all_leds_red(0);
        //leds.all_leds_red(1);
    }

    @Override
    public void execute() { // Los rollers giran hacia adentro para absorber el aro
        intakeSubsystem.setIntake(IntakeConstants.kIntakeSpeed);
        intakeSubsystem.setLoader(IntakeConstants.kLoaderSpeed);
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        System.out.println("Reload Cmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
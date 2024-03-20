package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeCmd extends Command { // This Command is a while-pressed Type
    private final IntakeSubsystem intakeSubsystem;
    private final double intakeSpeed;
    private final double loaderSpeed;
    //private TejuinoBoard leds = new TejuinoBoard();

    public IntakeCmd(IntakeSubsystem intakeSubsystem_i, double intakeSpeed_i, double loaderSpeed_i) {
        this.intakeSubsystem = intakeSubsystem_i;
        this.intakeSpeed = intakeSpeed_i;
        this.loaderSpeed = loaderSpeed_i;
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
        intakeSubsystem.setIntake(intakeSpeed);
        intakeSubsystem.setLoader(loaderSpeed);
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
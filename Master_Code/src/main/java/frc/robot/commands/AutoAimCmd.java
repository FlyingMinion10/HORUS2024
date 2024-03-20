package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeverSubsystem;
import frc.robot.subsystems.LimeLightSubsystem; // Asumiendo que tienes un subsistema para manejar la LimeLight

public class AutoAimCmd extends Command {
    private final LeverSubsystem leverSubsystem;
    private final LimeLightSubsystem limeLightSubsystem;
    //private TejuinoBoard leds = new TejuinoBoard();

    public AutoAimCmd( LimeLightSubsystem limeLightSubsystem_i, LeverSubsystem leverSubsystem_i) {
        this.limeLightSubsystem = limeLightSubsystem_i;
        this.leverSubsystem = leverSubsystem_i;
        addRequirements(limeLightSubsystem);
    }

    @Override
    public void initialize() {
        limeLightSubsystem.setPipeline(1); // Asumiendo que tienes un método para configurar la pipeline adecuada
        System.out.println("Auto Aim Started ");
        //leds.all_leds_green(0);
        //leds.all_leds_green(1);
    }

    @Override
    public void execute() {
        double angle = limeLightSubsystem.getTY(); // Obtiene la desviación angular (TY)
        leverSubsystem.setLeverSpeed(-angle); // Aplicar en el lever subsystem
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            leverSubsystem.stopLever();
        }
        System.out.println("Auto Aim Finished ");
    }

    @Override
    public boolean isFinished() {
        // Define una condición para finalizar el comando, por ejemplo, cuando el robot esté suficientemente alineado
        return false;
    }
}

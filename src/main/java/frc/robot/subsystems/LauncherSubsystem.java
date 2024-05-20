package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.LauncherConstants;


public class LauncherSubsystem extends SubsystemBase {

    private CANSparkMax frontUp_roller = new CANSparkMax(LauncherConstants.kFrontUpRollerMotorPort, MotorType.kBrushless);
    private CANSparkMax frontDown_roller = new CANSparkMax(LauncherConstants.kFrontDownRollerMotorPort, MotorType.kBrushless);
    private CANSparkMax backUp_roller = new CANSparkMax(LauncherConstants.kBackUpRollerMotorPort, MotorType.kBrushless);
    private CANSparkMax backDown_roller = new CANSparkMax(LauncherConstants.kBackDownRollerMotorPort, MotorType.kBrushless);


    public LauncherSubsystem() {
    }

    public void all_Rollers(double speed) { // Set speed de UpperRollers
        frontUp_roller.set(speed);
        frontDown_roller.set(-speed);
        backUp_roller.set(-speed);
        backDown_roller.set(speed);
    }
    
    public void front_Rollers(double speed) { // Set speed de UpperRollers
        frontUp_roller.set(speed);
        frontDown_roller.set(-speed);
    }
    
    public void back_Rollers(double speed) { // Set speed de UpperRollers
        backUp_roller.set(-speed);
        backDown_roller.set(speed);
    }

    public void stopRollers() {
        frontUp_roller.set(0);
        frontDown_roller.set(0);
        backUp_roller.set(0);
        backDown_roller.set(0);
    }
}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax intakeLeft_Motor = new CANSparkMax(IntakeConstants.kIntakeLeftMotorPort, MotorType.kBrushless);
    private CANSparkMax intakeRight_Motor = new CANSparkMax(IntakeConstants.kIntakeRightMotorPort, MotorType.kBrushless);
    private CANSparkMax loaderUp_Motor = new CANSparkMax(IntakeConstants.kLoaderUpMotorPort, MotorType.kBrushless);
    private CANSparkMax loaderDown_Motor = new CANSparkMax(IntakeConstants.kLoaderDownMotorPort, MotorType.kBrushless);    

    public IntakeSubsystem() {
    }


    public void setIntake(double speed) { // Set speed de UpperRollers
        intakeLeft_Motor.set(-speed);
        intakeRight_Motor.set(speed);
    }
    
    public void setLoader(double speed) { // Set speed de UpperRollers
        loaderUp_Motor.set(-speed);
        loaderDown_Motor.set(speed);
    }

    public void setBoth() {
        setIntake(IntakeConstants.kIntakeSpeed);
        setLoader(IntakeConstants.kLoaderSpeed);
    }

    public void stopIntake() {
        intakeLeft_Motor.set(0);
        intakeRight_Motor.set(0);
        loaderUp_Motor.set(0);
        loaderDown_Motor.set(0);
    }
}
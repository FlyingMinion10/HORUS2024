package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;
import frc.robot.Constants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase {

    private CANSparkMax ClimberLeft_Motor = new CANSparkMax(ClimberConstants.kClimberLeftMotorPort, MotorType.kBrushless);
    private CANSparkMax ClimberRight_Motor = new CANSparkMax(ClimberConstants.kClimberRightMotorPort, MotorType.kBrushless);

    RelativeEncoder ClimberLeft_Encoder = ClimberLeft_Motor.getEncoder(Type.kHallSensor, 42);
    RelativeEncoder ClimberRight_Encoder = ClimberRight_Motor.getEncoder(Type.kHallSensor, 42);
    
    

    public ClimberSubsystem() {
    }

    public void setClimberSpeed(double speed) { // Set speed de UpperRollers
        ClimberLeft_Motor.set(-speed);
        ClimberRight_Motor.set(speed);
    }

    public void stopClimber() {
        ClimberLeft_Motor.set(0);
        ClimberRight_Motor.set(0);
    }
}
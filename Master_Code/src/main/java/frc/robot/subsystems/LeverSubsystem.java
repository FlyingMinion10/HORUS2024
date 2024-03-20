package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;
import frc.robot.Constants.LeverConstants;


public class LeverSubsystem extends SubsystemBase {

    private final CANSparkMax leverMotor = new CANSparkMax(LeverConstants.kLeverMotorPort, MotorType.kBrushless);
    private final RelativeEncoder leverEncoder = leverMotor.getEncoder(Type.kHallSensor, 42);


    public LeverSubsystem() {
    }

    @Override
    public void periodic() {        
        SmartDashboard.putNumber("Lever position", getPos());
    }

    public void lResetEncoders() { // Set lever position to Zero
        leverEncoder.setPosition(0);
    }

    public void setLeverSpeed(double speed) { // Set speed de UpperRollers
        leverMotor.set(-speed);
    }

    public double getPos() {
        return -leverEncoder.getPosition();
    }
    
    public void SpecificPosition(int desiredPos) {

        if (leverEncoder.getPosition() < desiredPos) {
            setLeverSpeed(LeverConstants.kLeverSpeeed); 
            while (leverEncoder.getPosition() < desiredPos) { }
            stopLever();

        } else if (leverEncoder.getPosition() > desiredPos) {
            setLeverSpeed(-LeverConstants.kLeverSpeeed); 
            while (leverEncoder.getPosition() > desiredPos) { }
            stopLever();
        }
    }

    public void stopLever() {
        leverMotor.set(0);
    }
}
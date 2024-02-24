package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoAimCmd;
//import frc.robot.LimelightHelpers;

public class LimeLightSubsystem extends SubsystemBase {
    private final NetworkTable limeLightTable;
    private final double targetDistance = 70;
    private final double targetArea = 0.55;

    public LimeLightSubsystem() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        limeLightTable.getEntry("pipeline").setNumber(0); // Remove when its solved

        //post to smart dashboard periodically
    }

    @Override
    public void periodic() {        
        SmartDashboard.putNumber("Limelight TX", getYaw());
        SmartDashboard.putNumber("Limelight TY", getTY());
        SmartDashboard.putNumber("Limelight TA", getTA());
        SmartDashboard.putNumber("Limelight Distance", getDistance());
        SmartDashboard.putNumber("Limelight Error", getDistanceError());
        SmartDashboard.putNumber("Limelight ID", getID());
        SmartDashboard.updateValues();
    }

    public double getYaw() {
        return limeLightTable.getEntry("tx").getDouble(0);
    }
    
    public double getTY() {
        return limeLightTable.getEntry("ty").getDouble(0);
    }
    
    public double getTA() {
        return limeLightTable.getEntry("ta").getDouble(0);
    }
    
    public double getID() {
        return limeLightTable.getEntry("tid").getDouble(0);
    }
    
    public double getDistance() {
        // Este es un ejemplo. Necesitarás calcular la distancia basada en tu configuración específica.
        double targetOffsetAngle_Vertical = getTY();
        // Altura del objetivo - altura de la cámara sobre el suelo
        double targetHeight = 54; // Ejemplo: altura del AprilTag
        double cameraHeight = 20; // Ejemplo: altura de la cámara
        double cameraAngle = 20.5; // Ángulo de la cámara respecto al suelo
        
        return (targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle + targetOffsetAngle_Vertical));
    }

    public double getDistanceError() {
        return (getDistance() - targetDistance);
    }

    public double getAreaError() {
        return (getTA() - targetArea);
    }

    public boolean isTargetAligned() {
        // Este método debería verificar si el robot está alineado con el objetivo.
        // Por ejemplo, verificar si 'tx' y la distancia están dentro de umbrales aceptables.
        return  Math.abs(getDistanceError()) < 3 && Math.abs(getYaw()) < 2; // Ejemplo de umbral de yaw
    }

    public void setPipeline(int pipeline) {
        limeLightTable.getEntry("pipeline").setNumber(pipeline);
    }

    public void setLedSolid() {
        limeLightTable.getEntry("ledMode").setValue(0);
    }
    
    public void setLedOff() {
        limeLightTable.getEntry("ledMode").setValue(1);
    }

}

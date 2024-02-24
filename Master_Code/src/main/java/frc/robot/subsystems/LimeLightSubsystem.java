package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
//import frc.robot.LimelightHelpers;

public class LimeLightSubsystem extends SubsystemBase {
    private final NetworkTable limeLightTable;
    private final double targetDistance = 51.875;

    public LimeLightSubsystem() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");

        //post to smart dashboard periodically
        SmartDashboard.putNumber("Limelight TX", getYaw());
        SmartDashboard.putNumber("Limelight TY", getTY());
        SmartDashboard.putNumber("Limelight Distance", getDistance());
        SmartDashboard.putNumber("Limelight Error", getError());
        SmartDashboard.updateValues();
    }

    public double getYaw() {
        return limeLightTable.getEntry("tx").getDouble(0);
    }
    
    public double getTY() {
        return limeLightTable.getEntry("ty").getDouble(0);
    }
    
    public double getID() {
        return limeLightTable.getEntry("tid").getDouble(0);
    }
    
    public double getDistance() {
        // Este es un ejemplo. Necesitarás calcular la distancia basada en tu configuración específica.
        double targetOffsetAngle_Vertical = getTY();
        // Altura del objetivo - altura de la cámara sobre el suelo
        double targetHeight = 51.875; // Ejemplo: altura del AprilTag
        double cameraHeight = 16.93; // Ejemplo: altura de la cámara
        double cameraAngle = 28; // Ángulo de la cámara respecto al suelo
        
        return (targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle + targetOffsetAngle_Vertical));
    }

    public double getError() {
        return (getDistance() - targetDistance);
    }

    public boolean isTargetAligned() {
        // Este método debería verificar si el robot está alineado con el objetivo.
        // Por ejemplo, verificar si 'tx' y la distancia están dentro de umbrales aceptables.
        return Math.abs(getYaw()) < 1.0 && Math.abs(getError()) < 1; // Ejemplo de umbral de yaw
    }

    public void setPipeline(int pipeline) {
        limeLightTable.getEntry("pipeline").setNumber(pipeline);
    }

    /*public void setLedState() {
        limeLightTable.getEntry("ledMode").setValue(2);
    }*/

    public SwerveModuleState[] alignToTarget() {
        // Aquí, implementarás tu lógica para ajustar la orientación y la posición del robot.
        // Esto es solo un ejemplo simplificado.

        double ySpeed = getError() * 0.1; // Coeficiente para ajustar la velocidad basada en la distancia
        double xSpeed = getYaw() * 0.1; // Coeficiente para ajustar la velocidad de rotación basada en el yaw

        // Limita la velocidad y la velocidad de rotación para evitar valores extremos
        ySpeed = Math.min(ySpeed, 1.0);
        xSpeed = Math.min(xSpeed, 1.0);

        // Convierte a estados de módulos Swerve y aplica al robot
        SwerveModuleState[] AutoAimState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(ySpeed,xSpeed, 0.0));
        
        return AutoAimState;
    }
}

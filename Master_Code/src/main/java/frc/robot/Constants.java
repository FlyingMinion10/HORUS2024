package frc.robot;

//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {


    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //(MODIFY)
        public static final double kDriveMotorGearRatio = 1 / 8.14; //(MODIFY)
        public static final double kTurningMotorGearRatio = 1 / 21.4286; //(MODIFY)
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
        public static final double kITurning = 0;
        public static final double KDTurning = 0;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(23.25); //(MODIFY)
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(23.25); //(MODIFY)
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 24;
        public static final int kBackLeftDriveMotorPort = 9;
        public static final int kFrontRightDriveMotorPort = 17;
        public static final int kBackRightDriveMotorPort = 12;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 8;
        public static final int kFrontRightTurningMotorPort = 18;
        public static final int kBackRightTurningMotorPort = 13;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
        public static final int kBackRightDriveAbsoluteEncoderPort = 5;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 6;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 7;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
        
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.573242; 
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.150146;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.464111;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.707520;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.90; // Velocidad lineal maxima
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2; // Velocidad angular maxima
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; // 3
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
        public static final double kPXController = 0.5;
        public static final double kPYController = 0.5;
        public static final double kPThetaController = 0.05; //3
        public static final double kAutoRollerSpeed = 0.75;
        public static final double kWaitingTime = 5;
        

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kShooterControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverAutoTargetButtonIdx = 3;
        public static final int kDriverFieldOrientedButtonIdx = 6;

        public static final double kDeadband = 0.25;
    }

    public static final class LimeConstants {
        public static final double kDistanceToSpeaker = 55;
        public static final double kTargetSpeakertHeight = 57.125;

        public static final double kDistanceToAmp = 45;
        public static final double kTargetAmpHeight = 53.375;
        
        public static final double kDistanceToMic = 50; 
        public static final double kTargetMicHeight = 52; 
        
        public static final double kTargetFeedHeight = 20; 
    }

    public static final class LauncherConstants {
        public static final int kFrontUpRollerMotorPort = 5;
        public static final int kFrontDownRollerMotorPort = 6;  
        public static final int kBackUpRollerMotorPort = 3;
        public static final int kBackDownRollerMotorPort = 4;
        
        public static final double kRollers_Speed1 = 0.70; // 0.70
        public static final double kRollers_Speed2 = 0.60;
        public static final double kLowerRoller_Speed = 0.8;
        public static final double kFeederRollers_Speed = 0.2;
        public static final double kLoadingRollers_Speed = 0.2;
    }

    public static final class IntakeConstants {
        public static final int kIntakeLeftMotorPort = 7;
        public static final int kIntakeRightMotorPort = 11;

        public static final int kLoaderUpMotorPort = 14;
        public static final int kLoaderDownMotorPort = 15;
    
        public static final double kIntakeSpeed = 0.7;
        public static final double kLoaderSpeed = 0.3;
    }
    
    public static final class ClimberConstants {
        public static final int kClimberLeftMotorPort = 10;
        public static final int kClimberRightMotorPort = 16;
        
        public static final double kClimberSpeeed = 0.8;
    }

    public static final class LeverConstants {
        public static final int kLeverMotorPort = 2;
        public static final double kLeverSpeeed = 0.6; 
    }


}
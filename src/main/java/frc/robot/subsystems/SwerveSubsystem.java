package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.DriveConstants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;




public class SwerveSubsystem extends SubsystemBase {
    private final Field2d m_Field2d = new Field2d();

    private final SwerveModules frontLeft = new SwerveModules(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModules frontRight = new SwerveModules(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModules backLeft = new SwerveModules(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModules backRight = new SwerveModules(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(1, 0.0, 0.0), // Translation PID constants (2.5)
                    new PIDConstants(0.5, 0.0, 0.0), // Rotation PID constants (5.0)
                    5, // Max module speed, in m/s
                    0.4125, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> m_Field2d.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", m_Field2d);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public ChassisSpeeds getChassisSpeed() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
     }
     
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        
        positions[0] = frontLeft.getPosition();
        positions[1] = frontRight.getPosition();
        positions[2] = backLeft.getPosition();
        positions[0] = backRight.getPosition();
        return positions;
    }
    
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, gyro.getRotation2d(), getModulePositions()
    );

    // Reset robot's heading point
    public void zeroHeading() {
        sResetEncoders();
        gyro.reset();
    }
    
    // Set robot's heading point to 0 degrees
    public void changeHeading(int angle) {
        gyro.reset();
        gyro.setAngleAdjustment(angle);
    }

    // Reset encoder's Offsets
    public void sResetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
    
    // Get actual robot's heading
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    // CONFIG PATH PLANNER
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /*public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = frontLeft.getState();
        states[1] = frontRight.getState();
        states[2] = backLeft.getState();
        states[3] = backRight.getState();
        
        return states;
    }*/

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        double x = robotRelativeSpeeds.vxMetersPerSecond*-1;
        double y = robotRelativeSpeeds.vyMetersPerSecond*-1;
        double ø = robotRelativeSpeeds.omegaRadiansPerSecond*-1;
        
        ChassisSpeeds discretizeSpeeds = ChassisSpeeds.discretize(x, y , ø, 0.02);
        
        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discretizeSpeeds);
        setModulesState(targetStates);
    }

    @Override
    public void periodic() {        
        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        m_Field2d.setRobotPose(odometer.getPoseMeters());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    
    public void setModulesState(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

}
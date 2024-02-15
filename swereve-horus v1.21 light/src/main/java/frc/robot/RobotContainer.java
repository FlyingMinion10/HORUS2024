package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.LauncherActivateCmd;
import frc.robot.commands.LauncherFeederCmd;
import frc.robot.commands.LauncherLoadingCmd;
import frc.robot.Constants.LauncherConstants;

import frc.robot.subsystems.ClawSubsystem;
import frc.robot.commands.ClawCmd;
import frc.robot.Constants.ClawConstants;

import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;



public class RobotContainer {
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  private final ClawSubsystem clawSubsystem= new ClawSubsystem();
  //private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick shooterController = new Joystick(OIConstants.kShooterControllerPort);

  
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));


    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 2) // Resetea el frente del chasis (B)
            .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    new JoystickButton(shooterController, 1) // Activa el launcher (A)
            .whileTrue(new LauncherActivateCmd(launcherSubsystem, LauncherConstants.kUpperRoller_Speed1));
    new JoystickButton(shooterController, 2) // Activa el feeder (B)
            .whileTrue(new LauncherFeederCmd(launcherSubsystem, LauncherConstants.kFeederRollers_Speed));
    new JoystickButton(shooterController, 3) // Activa el loader (X) 
            .whileTrue(new LauncherLoadingCmd(launcherSubsystem, LauncherConstants.kLoadingRollers_Speed));
    
    new Trigger(() -> shooterController.getRawAxis(ClawConstants.kClawInFast_Button) > 0.5) // In Fast (LT)
            .whileTrue(new ClawCmd(clawSubsystem, ClawConstants.kClawInFast_Speed));
    new Trigger(() -> shooterController.getRawAxis(ClawConstants.kClawOutFast_Button) > 0.5) // Out Fast (RT)
            .whileTrue(new ClawCmd(clawSubsystem, ClawConstants.kClawOutFast_Speed));
    new JoystickButton(shooterController, ClawConstants.kClawInSlow_Button) // In slow (LB)
            .whileTrue(new ClawCmd(clawSubsystem, ClawConstants.kClawInSlow_Speed));
    new JoystickButton(shooterController, ClawConstants.kClawOutSlow_Button) // Out slow (RB)
            .whileTrue(new ClawCmd(clawSubsystem, ClawConstants.kClawOutSlow_Speed));
    
    
  }

  public Command getAutonomousCommand() {
    // Create trajectory 
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);

    // Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(-0.5, 0),
            new Translation2d(-0.5, 0.5)),
        new Pose2d(-0.1, 0.5, Rotation2d.fromDegrees(0)),
        trajectoryConfig);
    
    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
       trajectory,
       swerveSubsystem::getPose,
       DriveConstants.kDriveKinematics,
       xController,
       yController,
       thetaController,
       swerveSubsystem::setModulesState,
       swerveSubsystem);
    
    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      new InstantCommand(() -> swerveSubsystem.stopModules()));
      
  }
}

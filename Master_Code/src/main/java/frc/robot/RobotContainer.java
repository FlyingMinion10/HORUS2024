package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.Autonomous.AutonomousCentreCommand;
import frc.robot.Autonomous.AutonomousLeftCommand;
import frc.robot.Autonomous.AutonomousRightCommand;


import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.LauncherActivateCmd;
import frc.robot.commands.LauncherFeederCmd;
import frc.robot.commands.LauncherLoadingCmd;
import frc.robot.commands.ClawCmd;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LEDSubsystem;




public class RobotContainer {
  
  //private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  private final ClawSubsystem clawSubsystem= new ClawSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick shooterController = new Joystick(OIConstants.kShooterControllerPort);

  private static final double kNC = -1; // New constant to invert the cartesian in the autonomous
  
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    m_autoChooser.setDefaultOption("Autónomo Central", new AutonomousCentreCommand(swerveSubsystem, launcherSubsystem));
    m_autoChooser.addOption("Autónomo Derecho", new AutonomousRightCommand(swerveSubsystem, launcherSubsystem));
    m_autoChooser.addOption("Autónomo Izquierdo", new AutonomousLeftCommand(swerveSubsystem, launcherSubsystem));
    
    // Publica el selector en el SmartDashboard
    SmartDashboard.putData("Selector Autónomo", m_autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 2) // Resetea el frente del chasis (B)
            .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));


    new JoystickButton(shooterController, 1) // Activa el launcher 75 (A)
            .whileTrue(new LauncherActivateCmd(launcherSubsystem, LauncherConstants.kUpperRoller_Speed1));
    new JoystickButton(shooterController, 4) // Activa el launcher 85 (Y)
            .whileTrue(new LauncherActivateCmd(launcherSubsystem, LauncherConstants.kUpperRoller_Speed2));
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

    return m_autoChooser.getSelected();
      
  }
}

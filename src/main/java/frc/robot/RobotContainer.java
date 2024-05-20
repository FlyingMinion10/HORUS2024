package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants.LeverConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.commands.ClimberCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.LauncherActivateCmd;
import frc.robot.commands.LeverManualCmd;
import frc.robot.commands.SwerveJoystickCmd;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LeverSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

import frc.robot.Macros.LoadToAmp;
import frc.robot.Macros.LeverMac;
import frc.robot.Macros.ShooterForAutoMac;


public class RobotContainer {
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  private final LeverSubsystem leverSubsystem = new LeverSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick shooterController = new Joystick(OIConstants.kShooterControllerPort);
  private final SendableChooser<Command> p_autoChooser; //Autonomo del Path Planner
  
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      limeLightSubsystem,
      () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> driverJoystick.getRawButton(3),
      () -> driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

      //Autonomo del Path Planner
      NamedCommands.registerCommand("LauncherActivateCmd", new ShooterForAutoMac(launcherSubsystem, intakeSubsystem));
      NamedCommands.registerCommand("IntakeActivateCmd", new IntakeCmd(intakeSubsystem, IntakeConstants.kIntakeSpeed, IntakeConstants.kLoaderSpeed).withTimeout(2));
      NamedCommands.registerCommand("IntakeStopCmd", new InstantCommand(() -> intakeSubsystem.stopIntake()).withTimeout(0.2));
      //NamedCommands.registerCommand("IntakeMac", new IntakeMac(intakeSubsystem));
          
      p_autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", p_autoChooser);
        
      // Autonomo provisional

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 2) // Resetea el frente del chasis (B)
            .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    new JoystickButton(driverJoystick, 4) // Climber sube (POV 0)
            .whileTrue(new ClimberCmd(climberSubsystem, ClimberConstants.kClimberSpeeed));
    new JoystickButton(driverJoystick, 1) // Climber baja (POV 180)
            .whileTrue(new ClimberCmd(climberSubsystem, -ClimberConstants.kClimberSpeeed));


    new JoystickButton(shooterController, 1) // Activa el Shooter (A)
            .toggleOnTrue(new LauncherActivateCmd(launcherSubsystem, swerveSubsystem, intakeSubsystem));
    new JoystickButton(shooterController, 2) // Activa el Intake (B)
            .whileTrue(new IntakeCmd(intakeSubsystem, IntakeConstants.kIntakeSpeed, IntakeConstants.kLoaderSpeed));
    new JoystickButton(shooterController, 3) // Load to Amp (X)
            .toggleOnTrue(new LoadToAmp(intakeSubsystem, launcherSubsystem));
    new JoystickButton(shooterController, 4) // Score to Amp (Y)
            .toggleOnTrue(new LeverMac(leverSubsystem, launcherSubsystem));

    new POVButton(shooterController, 0) // Intake manual (POV 0)
            .whileTrue(new IntakeCmd(intakeSubsystem, 0, IntakeConstants.kLoaderSpeed));
    new POVButton(shooterController, 180) // Intake manual (POV 180)
            .whileTrue(new IntakeCmd(intakeSubsystem, 0, -IntakeConstants.kLoaderSpeed));
    new POVButton(shooterController, 90) // Intake manual positivo (POV 90)
            .whileTrue(new LeverManualCmd(leverSubsystem, LeverConstants.kLeverSpeeed));
    new POVButton(shooterController, 270) // Intake manual negativo (POV 270)
            .whileTrue(new LeverManualCmd(leverSubsystem, -LeverConstants.kLeverSpeeed));
    
    /*new JoystickButton(shooterController, 6) // ANGULAR FIXED POSITION (No usar)
            .toggleOnTrue(new LeverToPosCmd(leverSubsystem, 10));*/
  }

  public void m_resetEncoders() {
        swerveSubsystem.sResetEncoders();
        leverSubsystem.lResetEncoders();
  }

  public Command getAutonomousCommand() {
        
        return p_autoChooser.getSelected();



    //return p_autoChooser.getSelected();
  }
}

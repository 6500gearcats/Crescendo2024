// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.GetBestTarget;
import frc.robot.commands.RaiseArms;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.RaiseArms;
import frc.robot.commands.ShootNote;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveNormal;
import frc.robot.commands.DriveTurbo;
import frc.robot.commands.GetBestTarget;
import frc.robot.commands.MoveNeckUp;
import frc.robot.commands.MoveNeckDown;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private Vision visionSim;
  
private final Navigation m_vision = new Navigation();
private final Shooter m_robotShooter = new Shooter();
private final Intake m_robotIntake = new Intake();
private final Climber m_robotClimber = new Climber();
private final Neck m_Neck = new Neck();
  
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(OIConstants.kGunnerControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_vision.setDriveController(m_robotDrive);

    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1), //0.1
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1), //0.1
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1),
                !m_driverController.getRightBumper()),
            m_robotDrive));
      
  }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Turbo Buttons 
    new JoystickButton(m_driverController, Button.kLeftBumper.value).whileTrue(new DriveTurbo(m_robotDrive));
    new JoystickButton(m_driverController, Button.kLeftBumper.value).onFalse(new DriveNormal(m_robotDrive));

    // Set the wheels in locked arrangement to prevent movement
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new GetBestTarget(m_vision, m_robotDrive));
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new ShootNote(m_robotShooter, m_robotIntake));
    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new PickUpNote(m_robotIntake));
    new JoystickButton(m_driverController, Button.kBack.value)
        .whileTrue(new RaiseArms(m_robotClimber));

    new Trigger(() -> ( m_driverController.getLeftTriggerAxis() > 0.5))
        .whileTrue(new RunCommand(() -> m_robotShooter.setShooterSpeedFast(), m_robotShooter));
    new Trigger(() -> ( m_driverController.getRightTriggerAxis() > 0.5))
        .whileTrue(new RunCommand(() -> m_robotIntake.setFeedSpeed(), m_robotIntake));

    // Basic Functions 
    new Trigger(() -> (m_gunnerController.getRightTriggerAxis() > 0.5))
      .whileTrue(new ShootNote(m_robotShooter, m_robotIntake));

    new JoystickButton(m_gunnerController, Button.kY.value)
        .whileTrue(new MoveNeckUp(m_Neck));
    new JoystickButton(m_gunnerController, Button.kA.value)
        .whileTrue(new MoveNeckDown(m_Neck));
    
  }
  public Command getAutonomousCommand() {
    PathPlannerPath Demo_Path = PathPlannerPath.fromPathFile("Demo_Path");
    return AutoBuilder.followPath(Demo_Path);

    // Code to use an Auto
    // return new PathPlannerAuto("Example Path");
}

}
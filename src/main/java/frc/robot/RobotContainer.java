// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.JDBCType;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.GetBestTarget;
import frc.robot.commands.MoveToClosestNote;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.ShootNote;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //private Vision visionSim;
  
private Vision m_simVision = new Vision();
private final Navigation m_vision = new Navigation();
private final Shooter m_robotShooter = new Shooter();
private final Intake m_robotIntake = new Intake();
private final NoteFinder m_NoteFinder = new NoteFinder(m_simVision);
private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_simVision);

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

    // Set the wheels in locked arrangement to prevent movement
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new GetBestTarget(m_vision, m_robotDrive));
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new ShootNote(m_robotShooter, m_robotIntake));
    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new PickUpNote(m_robotIntake));

    new Trigger(() -> ( m_driverController.getLeftTriggerAxis() > 0.5))
        .whileTrue(new RunCommand(() -> m_robotShooter.setShooterSpeedFast(), m_robotShooter));
    new Trigger(() -> ( m_driverController.getRightTriggerAxis() > 0.5))
        .whileTrue(new RunCommand(() -> m_robotIntake.setFeedSpeed(), m_robotIntake));

    // Basic Functions 
    new Trigger(() -> (m_gunnerController.getRightTriggerAxis() > 0.5))
      .whileTrue(new ShootNote(m_robotShooter, m_robotIntake));

    new JoystickButton(m_gunnerController, Button.kA.value)
        .whileTrue(new MoveToClosestNote(m_NoteFinder, m_robotDrive, m_robotIntake));

  }
  public Command getAutonomousCommand() {
    PathPlannerPath Demo_Path = PathPlannerPath.fromPathFile("Demo_Path");
    return AutoBuilder.followPath(Demo_Path);

    // Code to use an Auto
    // return new PathPlannerAuto("Example Path");
}

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.VisionConstants.kCameraNameNote;
import static frc.robot.Constants.VisionConstants.kCameraNameTag;

import java.sql.JDBCType;
import java.util.ArrayList;
import java.util.HashMap;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.GetBestTarget;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.ShootNote;
import frc.robot.commands.climb.LowerHooks;
import frc.robot.commands.climb.RaiseHooks;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.GetBestTarget;
import frc.robot.commands.GetChosenTarget;
import frc.robot.commands.GrabNote;
import frc.robot.commands.MoveToClosestNote;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.ShootNote;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BackwardsIntake;
import frc.robot.commands.DriveNormal;
import frc.robot.commands.DriveTurbo;
import frc.robot.commands.GetBestTarget;
import frc.robot.commands.MoveNeckDown;
import frc.robot.commands.MoveNeckUp;
import frc.robot.commands.NeckStable;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.NoteFinder;
import frc.robot.subsystems.Shooter;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  //private Vision visionSim;

private PhotonCamera cameraTag = new PhotonCamera(kCameraNameTag);
private PhotonCamera cameraNote = new PhotonCamera(kCameraNameNote);
private Vision m_tagVision = new Vision(cameraTag);
private Vision m_noteVision = new Vision(cameraNote);

private final Navigation m_vision = new Navigation(m_tagVision);
private final Shooter m_robotShooter = new Shooter();
private final Intake m_robotIntake = new Intake();
private final Climber m_robotClimber = new Climber();
private final NoteFinder m_NoteFinder = new NoteFinder(m_noteVision);
private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_tagVision);
private final Neck m_Neck = new Neck();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(OIConstants.kGunnerControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Named commands must be registered before the creation of any PathPlanner Autos or Paths.
    NamedCommands.registerCommand("DemoCommand", Commands.print("Ran Demo Command"));
    NamedCommands.registerCommand("ShootNote", new ShootNote(m_robotShooter, m_robotIntake).withTimeout(1.5));
    NamedCommands.registerCommand("RunIntake", new PickUpNote(m_robotIntake));
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

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

    m_Neck.setDefaultCommand(new NeckStable(m_Neck));  
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
        .onTrue(new PickUpNote(m_robotIntake).andThen(new WaitCommand(.2))
        .andThen(new BackwardsIntake(m_robotIntake).withTimeout(.2)));

    new Trigger(() -> ( m_driverController.getLeftTriggerAxis() > 0.5))
        .whileTrue(new RunCommand(() -> m_robotShooter.setShooterSpeedFast(), m_robotShooter));
    new Trigger(() -> ( m_driverController.getRightTriggerAxis() > 0.5))
        .whileTrue(new RunCommand(() -> m_robotIntake.setFeedSpeed(), m_robotIntake));

    //Gunner controls

    new JoystickButton(m_gunnerController, Button.kLeftBumper.value)
        .whileTrue(new BackwardsIntake(m_robotIntake));

    new JoystickButton(m_gunnerController, Button.kRightBumper.value)
        .whileTrue(new GetChosenTarget(m_noteVision, m_robotDrive));

    new JoystickButton(m_gunnerController, Button.kX.value)
        .onTrue(new RaiseHooks(m_robotClimber));
    new JoystickButton(m_gunnerController, Button.kB.value)
        .onTrue(new LowerHooks(m_robotClimber));


    // Basic Functions 
    new Trigger(() -> (m_gunnerController.getRightTriggerAxis() > 0.5))
      .whileTrue(new ShootNote(m_robotShooter, m_robotIntake));
      
    new JoystickButton(m_gunnerController, Button.kY.value)
        .whileTrue(new MoveNeckUp(m_Neck));
    new JoystickButton(m_gunnerController, Button.kA.value)
        .whileTrue(new MoveNeckDown(m_Neck));
    
    new Trigger(() -> (m_gunnerController.getLeftTriggerAxis() > 0.5))
        .onTrue (new GrabNote(m_NoteFinder, m_robotDrive, m_robotIntake));

  }

  public Command getAutonomousCommand() {
    /**Code to run a singular path
  PathConstraints constraints = new PathConstraints(
    3.0, 3.0,
    Units.degreesToRadians(540), Units.degreesToRadians(720));


    PathPlannerPath Demo_Path = PathPlannerPath.fromPathFile("Demo_Path");
    return AutoBuilder.pathfindThenFollowPath(Demo_Path, constraints);
    Code to use an Auto
    return new PathPlannerAuto("TopAuto");
    */
    return autoChooser.getSelected();
}

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
/* *
import com.pathplanner.lib.PathConstraints; import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
*/
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
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
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(OIConstants.kGunnerControllerPort);

  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    

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

    //DRIVER CONTROLLER
    //while left button is pressed, speed is modified by the turbo mode modifier constant 
    //new JoystickButton(m_driverController, Button.kLeftBumper.value).whileTrue(new DriveTurbo(m_robotDrive));
    //new JoystickButton(m_driverController, Button.kLeftBumper.value).onFalse(new DriveNormal(m_robotDrive));
    //Turn on lights: Yellow = Back,     Purple = Start
    //new JoystickButton(m_driverController, Button.kBack.value).whileTrue(new LightYellow());
    //new JoystickButton(m_driverController, Button.kStart.value).whileTrue(new LightPurple());

    // Set the wheels in locked arrangement to prevent movement
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));


    //GUNNER CONTROLLER
    //sets the left stick to move arm up, increasing in speed with how far the joystick is pushed
    //new Trigger(() -> m_gunnerController.getLeftY() > 0).whileTrue(new ArmUpWithSpeed(m_Arm, (ArmConstants.kArmForwardMaxSpeed * m_gunnerController.getLeftY())));
    //sets the left stick to move arm down, increasing in speed with how far the joystick is pushed
    //new Trigger(() -> m_gunnerController.getLeftY() < 0).whileTrue(new ArmDownWithSpeed(m_Arm, (ArmConstants.kArmReverseMaxSpeed * m_gunnerController.getLeftY())));
  
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
   // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    //ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("fullAuto", new PathConstraints(4, 3));
    // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.
    //HashMap<String, Command> eventMap = new HashMap<>();
    //eventMap.put("marker1", new PrintCommand("Passed marker 1"));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    /*
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        m_robotDrive::getPose, // Pose2d supplier
        m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        DriveConstants.kDriveKinematics,
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        m_robotDrive::setModuleStates,// Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
    );
    */

    // Run path following command, then stop at the end.
    //return fullAuto.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    /*  if (DriverStation.getLocation() == 1)
    {
      System.out.println("Starting Path 1");
      return new CloseClaw(m_Claw).withTimeout(0.5)
      .andThen(new MoveArmToPosition(ArmConstants.kArmHighAngle, m_Arm)).withTimeout(5.0)
      .andThen(cubePath1_1
      .andThen(new OpenClaw(m_Claw).withTimeout(0.5))      
      .andThen(cubePath1_2
      .andThen(new MoveArmToPosition(ArmConstants.kArmStowAngle, m_Arm)).withTimeout(5.0)
      .andThen(cubePath1_3
      .andThen(()-> m_robotDrive.drive(0, 0, 0, false
      )))));
    }
    else if (DriverStation.getLocation() == 2)
    {
      System.out.println("Starting Path 2");
      return new CloseClaw(m_Claw).withTimeout(0.5)
      .andThen(new MoveArmToPosition(ArmConstants.kArmHighAngle, m_Arm)).withTimeout(5.0)
      .andThen(cubePath2_1
      .andThen(new OpenClaw(m_Claw).withTimeout(0.5))      
      .andThen(cubePath2_2
      .andThen(new MoveArmToPosition(ArmConstants.kArmStowAngle, m_Arm)).withTimeout(5.0)
      .andThen(cubePath2_3
      .andThen(pathEnd1
      .andThen(new ClimbPlatform(m_robotDrive)
      .andThen(()-> m_robotDrive.drive(0, 0, 0, false
      )))))));
    }
    else if (DriverStation.getLocation() == 3)
    {
      System.out.println("Starting Path 3");
      return new CloseClaw(m_Claw).withTimeout(0.5)
      .andThen(new MoveArmToPosition(ArmConstants.kArmHighAngle, m_Arm)).withTimeout(5.0)
      .andThen(cubePath3_1
      .andThen(new OpenClaw(m_Claw).withTimeout(0.5))      
      .andThen(cubePath3_2
      .andThen(new MoveArmToPosition(ArmConstants.kArmStowAngle, m_Arm)).withTimeout(5.0)
      .andThen(cubePath3_3
      .andThen(()-> m_robotDrive.drive(0, 0, 0, false
      )))));
    }
    else
    {
      return new WaitCommand(0);
    }
    */
  }


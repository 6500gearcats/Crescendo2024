// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Navigation extends SubsystemBase {
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // Change this to match the name of your camera
    // CHANGED CAMERA NAME TO A CONSTANT

    //static PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    //static PhotonCamera camera = new PhotonCamera(Constants.kCamName);

    // PID constants should be tuned per robot
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    private DriveSubsystem m_drive;

    // Create a new gyro object

    private AHRS m_gyro;

    private int m_gyroSim;
    private SimDouble m_simAngle;
    private SimBoolean m_connected;
    private SimBoolean m_calibrating;
  

    

  public double getRotation()
  {
    var result = m_drive.getLatestCameraResult();
    double rotation;
    if (result.hasTargets()) 
    {
      // Calculate angular turn power
      // Remove -1.0 because it was inverting results.
      rotation = -turnController.calculate(result.getBestTarget().getYaw(), 0) * Constants.kRangeSpeedOffset;

  } else {
      // If we have no targets, stay still.
      rotation = 0;
  }
    return rotation;
  }

  // HEADER - METHOD TO GET ROTATION FOR A CHOSEN TARGET

  public double getChosenTargetRotation(int targetID)
  {
    var result = m_drive.getLatestCameraResult();
    // Get a list of all of the targets that have been detected. 
    List<PhotonTrackedTarget> targets = result.getTargets();
    double rotation = 0;

    // For each target we have check if it matches the id you want.
    for(PhotonTrackedTarget target : targets)
    {
      if(result.hasTargets())
      {
        if(target.getFiducialId() == targetID)
        {
          // Use the value of target to find our rotation using the getYaw command
          rotation = -turnController.calculate(target.getYaw(), 0) * Constants.kRangeSpeedOffset;
        }
      }
      else
      {
        rotation = 0;
      }
    }

    return rotation;
  }

  // HEADER - METHOD TO FIND SPEED TO APROACH A TARGET
  public double getRange()
  {
    var result = m_drive.getLatestCameraResult();
    double range;
    if (result.hasTargets()) {
                // First calculate range
                range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS, // Previously declarde
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // THE FOLLOWING EQUATION CAN BE USED TO CALCULATE FORWARD SPEED
                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                double forwardSpeed = -DriveSubsystem.turnController.calculate(range, GOAL_RANGE_METERS);
                return forwardSpeed * Constants.kRangeSpeedOffset;
            } else {
                // If we have no targets, stay still.
                return 0;
                // When this is implemented - DO NOTHING IF RANGE IS 0
            }
  }

  // HEADER - METHOD TO FIND SPEED TO APROACH A TARGET
  public double getChosenTargetRange(int targetID)
  {
    var result = m_drive.getLatestCameraResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    double range = 0;
    double forwardSpeed = 0;

    for(PhotonTrackedTarget target : targets)
    {
      if(result.hasTargets())
      {
        range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS, // Previously declarde
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(target.getPitch()));

                // THE FOLLOWING EQUATION CAN BE USED TO CALCULATE FORWARD SPEED
                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                forwardSpeed = -DriveSubsystem.turnController.calculate(range, GOAL_RANGE_METERS);
                forwardSpeed *= Constants.kRangeSpeedOffset;
      }
      else
      {
        // If we have no targets, stay still
        range = 0;
      }
    }

    return forwardSpeed; 
  }

  public Pose3d getRobotPosition()
  {
    // Create a pose3d object for our output
    Pose3d robotPose = null;

    // Create an april tag field layout object
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // Create a result object
    var result = m_drive.getLatestCameraResult();
    // Create a target object using the values of the result object
    PhotonTrackedTarget target = result.getBestTarget();
      
    Transform3d camToRobot = new Transform3d(new Translation3d(0, 0, CAMERA_HEIGHT_METERS), new Rotation3d(0, CAMERA_PITCH_RADIANS, 0));
    
    Optional<Pose3d> fieldRelativeAprilTagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
    
      
    if(fieldRelativeAprilTagPose.isPresent())
    {
      // Use the target and april tag layout to determine the position of the robot
      // FOLLOWING CODE USING UNDIFINED TRANSFORM 3D PARAMETER, PLEASE FIND HOW TO GET
      robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), fieldRelativeAprilTagPose.get(), camToRobot);
    }
    // Return the value of the robot's position
    return robotPose;
  }

  /** Creates a new Navigation object when used. */
  public Navigation() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setDriveController(DriveSubsystem robotDrive) {
    m_drive = robotDrive;
  }
}

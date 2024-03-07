// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.Constants.VisionConstants;

public class Navigation extends SubsystemBase {
    

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

    DriveSubsystem m_drive;
    Vision m_vision;

  public double getRotation()
  {
    var result = m_vision.getLatestCameraResult();
    double rotation;
    if (result.hasTargets()) 
    {
      // Calculate angular turn power
      // Remove -1.0 because it was inverting results.
      rotation = turnController.calculate(result.getBestTarget().getYaw(), 0) * Constants.kRangeSpeedOffset;

  } else {
      // If we have no targets, stay still.
      rotation = 0;
  }
    return rotation;
  }

  // HEADER - METHOD TO FIND DISTANCE FROM TARGET
  public double getRange()
  {
    var result = m_vision.getLatestCameraResult();
    double range;
    if (result.hasTargets()) {
                // First calculate range
                range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                VisionConstants.CAMERA_HEIGHT_METERS, // Previously declarde
                                VisionConstants.TARGET_HEIGHT_METERS,
                                VisionConstants.CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // THE FOLLOWING EQUATION CAN BE USED TO CALCULATE FORWARD SPEED
                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                double forwardSpeed = -DriveSubsystem.turnController.calculate(range, VisionConstants.GOAL_RANGE_METERS);
                return forwardSpeed * Constants.kRangeSpeedOffset;
            } else {
                // If we have no targets, stay still.
                return 0;
                // When this is implemented - DO NOTHING IF RANGE IS 0
            }
  }

  

  

  /** Creates a new Navigation object when used. */
  public Navigation(Vision vision) {
    m_vision = vision;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_vision.getLatestCameraResult().hasTargets())
    {
    SmartDashboard.putNumber("Current Fiducial Id:", m_vision.getLatestCameraResult().getBestTarget().getFiducialId());
    }
  }

  public void setDriveController(DriveSubsystem robotDrive) {
    m_drive = robotDrive;
  }

  public Pose3d getRobotPosition()
  {
    // Create a pose3d object for our output
    Pose3d robotPose = null;

    // Create an april tag field layout object
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // Create a result object
    var result = m_vision.getLatestCameraResult();
    // Create a target object using the values of the result object
    PhotonTrackedTarget target = result.getBestTarget();
      
    // This tells the robot where the camera is. VERY IMPORTANT FOR ACCURACY.
    // TODO: We need to test this.
    Transform3d camToRobot = new Transform3d(new Translation3d(0, VisionConstants.VISION_CAMERA_Y_OFFSET, VisionConstants.CAMERA_HEIGHT_METERS), new Rotation3d(0, VisionConstants.CAMERA_PITCH_RADIANS, VisionConstants.VISION_CAMERA_YAW_RADIANS));
    
    Optional<Pose3d> fieldRelativeAprilTagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
    
      
    if(fieldRelativeAprilTagPose.isPresent() && result.hasTargets())
    {
      // Use the target and april tag layout to determine the position of the robot
      // FOLLOWING CODE USING UNDIFINED TRANSFORM 3D PARAMETER, PLEASE FIND HOW TO GET
      robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), fieldRelativeAprilTagPose.get(), camToRobot);
    }
    // Return the value of the robot's position
    return robotPose;
  }

}

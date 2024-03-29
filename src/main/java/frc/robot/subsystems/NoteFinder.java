// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.kCameraNameNote;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vision;

public class NoteFinder extends SubsystemBase {
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(12);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(0);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(0);

    // Change this to match the name of your camera
    // CHANGED CAMERA NAME TO A CONSTANT

    //static PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    //static PhotonCamera camera = new PhotonCamera(Constants.kCamName);

    // PID constants should be tuned per robot
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    final double ANGULAR_P = 0.05;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    DriveSubsystem m_drive;
    Vision m_vision;

  public double getRotation()
  {
    var result = m_vision.getLatestResult();
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

  // HEADER - METHOD TO FIND DISTANCE FROM TARGET
  public double getRange()
  {
    var result = m_vision.getLatestResult();
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
  
// This is a method it get distance

  public double getDistance()
  {
    var result = m_vision.getLatestResult();
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
            } else {
                // If we have no targets, stay still.
                return 0;
                // When this is implemented - DO NOTHING IF RANGE IS 0
            }
      return range;
  }

  /** Creates a new Navigation object when used. */
  public NoteFinder(Vision vision) {
    m_vision = vision;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Sim-Robot (Vision) Speed", getRange());
    SmartDashboard.putNumber("Sim-Robot (Vision) Rotation", getRotation());
  }

  public void setDriveController(DriveSubsystem robotDrive) {
    m_drive = robotDrive;
  }

  public int NearestNotePosition() {
    return 1; // Placeholder for when vision is utilized in here to find a notes position
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.Constants.VisionConstants;

public class Navigation extends SubsystemBase {

  // Change this to match the name of your camera
  // CHANGED CAMERA NAME TO A CONSTANT

  // static PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  // static PhotonCamera camera = new PhotonCamera(Constants.kCamName);

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  Vision m_vision;

  private Optional<Alliance> m_team;
  private DriveSubsystem m_drive;

  public double getRotation() {
    var result = m_vision.getLatestCameraResult();
    double rotation;
    if (result.hasTargets()) {
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
  public double driveToTarget()
  {
    var result = m_vision.getLatestCameraResult();
    double range;
    if (result.hasTargets()) {
      // First calculate range
      range = PhotonUtils.calculateDistanceToTargetMeters(
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

  /** Creates a new Navigation object when used. 
 * @param m_robotDrive */
  public Navigation(Vision vision, DriveSubsystem robotDrive) {
    m_vision = vision;
    m_drive = robotDrive;
    m_team = DriverStation.getAlliance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_vision.getLatestCameraResult().hasTargets()) {
      // SmartDashboard.putNumber("Current Fiducial Id:",
      // m_vision.getLatestCameraResult().getBestTarget().getFiducialId());
    }
  }

  public void setDriveController(DriveSubsystem robotDrive) {
    m_drive = robotDrive;
  }

  public boolean inWing() {

    double x = m_drive.getPose().getX();
    //if (m_team.isEmpty()) 
      m_team = DriverStation.getAlliance();
    
    if (m_team.isPresent()) {
      if((m_team.get() == DriverStation.Alliance.Blue) && x < 6.5)
      {
        return true;
      }
      else if ((m_team.get() == DriverStation.Alliance.Red) && x > 11.3)
      {
        return true;
      }
    }
    return false;

  }

}

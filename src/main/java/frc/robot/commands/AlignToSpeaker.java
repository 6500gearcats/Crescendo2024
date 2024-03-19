// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToSpeaker extends Command {

  private static int kRedSpeaker = 4;
  private static int kBlueSpeaker = 7;

  private int m_target = 0;
  private double m_targetAngle =0.0;
  private double m_targetDistance =0.0;

  private PIDController turnController;

  Vision m_vision;
  DriveSubsystem m_drive;

  double d = 0;

  /** Creates a new AlignToSpeaker. */
  public AlignToSpeaker(Vision theVision, DriveSubsystem theDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = theVision;
    m_drive = theDrive;
    addRequirements(m_drive);
    
    turnController = new PIDController(2.0, 0.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      var myAlliance = alliance.get();
      if ( myAlliance.equals(DriverStation.Alliance.Red) ) {
        m_target = kRedSpeaker;
      }
      else if ( myAlliance.equals(DriverStation.Alliance.Blue) ) {
        m_target = kBlueSpeaker;
      }

      m_targetAngle = m_vision.getChosenTargetRotation(m_target);
      m_targetDistance = m_vision.getChosenTargetDistance(m_target);
    }


    if ((m_target==0) || closeEnough(m_targetAngle) ) {
      m_target = 0;
      this.cancel();
    }
  }

  private boolean closeEnough(double target) {
    return Math.abs(target) < 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotation;
    double bSquared;
    double a = -9;
    double b;
    double c;

    // Returns degrees

    Pose2d now = m_drive.getPose();
    double robotAngle = now.getRotation().getDegrees();
    //Update our target angle based on robot movement feedback
    m_targetAngle += robotAngle; // robot angle is +ve CCW

    rotation = m_targetAngle;

    if (rotation != 0) {

      c = m_targetDistance;
      bSquared = Math.pow(a, 2) + Math.pow(c, 2) - 2 * (c * a) * Math.cos(rotation + 90);
      b = Math.sqrt(bSquared);
      d = Math.asin((Math.sin(rotation + 90) * a) / b);
      rotation -= d;
    }

    rotation = turnController.calculate(rotation, m_targetAngle);
    SmartDashboard.putNumber("Align to Speaker Rotation", rotation);

    if (closeEnough(Math.abs(rotation))) {
      rotation = 0;
      m_targetAngle = 0;
    }

    m_drive.drive(0, 0, -rotation * .01, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Pose2d now = m_drive.getPose();
    // double rot = now.getRotation().getDegrees();
    
    return closeEnough(m_targetAngle + d);

  }
}

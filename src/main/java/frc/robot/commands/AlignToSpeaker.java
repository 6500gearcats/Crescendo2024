// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToSpeaker extends Command {

  private static int kRedSpeaker = 4;
  private static int kBlueSpeaker = 7;

  private int m_target = 0;

  Vision m_vision;
  DriveSubsystem m_drive;

  double d = 0;

  /** Creates a new AlignToSpeaker. */
  public AlignToSpeaker(Vision theVision, DriveSubsystem theDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = theVision;
    m_drive = theDrive;
    addRequirements(m_drive);
    
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
    }
    else {
      m_target = 0;
      this.cancel();
    }
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

    rotation = m_vision.getChosenTargetRotation(m_target);

    if (rotation != 0) {

      c = m_vision.getChosenTargetDistance(m_target);
      bSquared = Math.pow(a, 2) + Math.pow(c, 2) - 2 * (c * a) * Math.cos(rotation + 90);
      b = Math.sqrt(bSquared);
      d = Math.asin((Math.sin(rotation + 90) * a) / b);
      rotation -= d;
    }

    if (Math.abs(rotation) > .1) {
      SmartDashboard.putNumber("Align to Speaker Rotation", rotation);
    } else {
      rotation = 0;
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

    var rot = Math.abs(m_vision.getChosenTargetRotation(m_target));
    return (rot + d) < 1;

  }
}

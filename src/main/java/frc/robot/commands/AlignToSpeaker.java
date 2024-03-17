// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToSpeaker extends Command {
  Vision m_vision;
  DriveSubsystem m_drive;
  /** Creates a new AlignToSpeaker. */
  public AlignToSpeaker(Vision theVision, DriveSubsystem theDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = theVision;
    m_drive = theDrive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = 0;
    var alliance = DriverStation.getAlliance();

    double bSquared;
    double a = -11;
    double b;
    double c;

    if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
    {
      // Returns degrees
      rotation = m_vision.getChosenTargetRotation(3);
      c = m_vision.getChosenTargetDistance(3);
      bSquared = Math.pow(a, 2) + Math.pow(c, 2) -2 * (c*a) * Math.cos(rotation + 90);
      b = Math.sqrt(bSquared);
      rotation -= Math.asin((Math.sin(rotation + 90) *a)/b);
    }
    else
    {
      // Returns degrees
      
      rotation = m_vision.getChosenTargetRotation(3);
      c = m_vision.getChosenTargetDistance(7);
      bSquared = Math.pow(a, 2) + Math.pow(c, 2) -2 * (c*a) * Math.cos(rotation + 90);
      b = Math.sqrt(bSquared);
      rotation -= Math.asin((Math.sin(rotation + 90) *a)/b);
    }

    if(Math.abs(rotation) > .1)
    {
      SmartDashboard.putNumber("Align to Speaker Rotation", rotation);
    }
    else
    {
      rotation = 0;
    }

    m_drive.drive(0, 0, -rotation * .05, false);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var alliance = DriverStation.getAlliance();

    // Align to red or blue amp based on alliance
    if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) 
    {
      return m_vision.getChosenTargetRotation(3) == 0;
    }
    else
    {
      return m_vision.getChosenTargetRotation(7) == 0;
    }

  }
}

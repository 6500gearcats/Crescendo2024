// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
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
    double rotation;
    var alliance = DriverStation.getAlliance();

    if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) // TODO Best logic
    {
      rotation = m_vision.getChosenTargetRotation(3) + Units.inchesToMeters(-11)/Math.acos(m_vision.getChosenTargetRange(3));
    }
    else
    {
      rotation = m_vision.getChosenTargetRotation(7) + Units.inchesToMeters(-11)/Math.acos(m_vision.getChosenTargetRange(7));
    }

    m_drive.drive(0, 0, -rotation * .5, false);

    if(Math.abs(rotation) < .001)
    {
      SmartDashboard.putNumber("Align to Speaker Rotation", rotation); 
    }
    else
    {
      rotation = 0;
    }
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.DriveSubsystem;

public class LineUpAmp extends Command {
  public final DriveSubsystem m_drive;

  public LineUpAmp(DriveSubsystem theDrive) {
    m_drive = theDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new PathFindToPos(1.82, 7.72, Rotation2d.fromDegrees(-90), m_drive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getPose() == new Pose2d(1.82, 7.72, Rotation2d.fromDegrees(-90));
  }
}

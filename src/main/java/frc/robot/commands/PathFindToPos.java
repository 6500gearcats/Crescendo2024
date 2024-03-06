// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class PathFindToPos extends Command {
  private int x;
  private int y;
  private Rotation2d Rotation;
  private PathConstraints constraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  Pose2d targetPose;
  DriveSubsystem m_drive;
  /** Creates a new PathFindToPos. */
  public PathFindToPos(int posX, int posY, Rotation2d rot, DriveSubsystem drive) {
    x = posX;
    y = posY;
    m_drive = drive;
    Rotation = rot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(180));
Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getPose()==targetPose;
  }
}

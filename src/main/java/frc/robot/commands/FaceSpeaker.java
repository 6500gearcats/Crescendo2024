// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;
import frc.robot.Vision;

public class FaceSpeaker extends Command {
  private DriveSubsystem m_drive;
  private Vision m_vision;

  /** Creates a new FaceSpeaker. */
  public FaceSpeaker(DriveSubsystem theDrive, Vision theVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = theDrive;
    m_vision = theVision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double rotation = m_vision.getChosenTargetRotation(5);
    m_drive.drive(0, 0, rotation, true);
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
    double rotation = m_vision.getChosenTargetRotation(5);
    return rotation < 0.1 && rotation > -0.1;
  }
}

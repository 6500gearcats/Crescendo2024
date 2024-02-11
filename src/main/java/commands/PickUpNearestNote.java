// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Navigation;

public class PickUpNearestNote extends Command {
  /** Creates a new PickUpNearestNote. */
  private final Navigation m_vision;
  private final DriveSubsystem m_drive;
  private final Intake m_intakeSystem;

  public PickUpNearestNote(Navigation vision, DriveSubsystem drive, Intake intake) {
    m_vision = vision;
    m_drive = drive;
    m_intakeSystem = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Use Vision to find note
    // Get Values of the note to use for PathFindToPos
    new PathFindToPos(0, 0);
    new PickUpNote(m_intakeSystem);
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
    return false;
  }
}

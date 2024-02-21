// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootNoteConstants;
import frc.robot.subsystems.Intake;

public class PickUpNote extends Command {
  /** Creates a new PickUpNote. */
  private final Intake m_IntakeSystem;
  
  public PickUpNote(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSystem = intake;
    addRequirements(m_IntakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSystem.setPickupSpeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_IntakeSystem.NoteIsPresent();
  }

  
}

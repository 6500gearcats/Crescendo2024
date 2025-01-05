// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Neck;

/** Add your docs here. */
public class NoteInPlace  extends Command  {

private final Intake m_IntakeSystem;

  public NoteInPlace(Intake theIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSystem = theIntake;
    addRequirements(m_IntakeSystem);
  }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {
     m_IntakeSystem.setPickupSpeedSlow();
   }

   // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.stop();
  }

   // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IntakeSystem.NoteInPlace();
  }

}

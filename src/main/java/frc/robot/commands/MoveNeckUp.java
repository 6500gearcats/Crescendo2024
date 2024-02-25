// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NeckConstants;
import frc.robot.subsystems.Neck;

public class MoveNeckUp extends Command {
  private final Neck m_Neck;

  public MoveNeckUp(Neck theNeck) {
    m_Neck = theNeck;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Neck.getMotorController().set(NeckConstants.kNeckForwardSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Neck.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Neck.AtMaxHeight();
  }
}

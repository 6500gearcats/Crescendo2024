// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NeckConstants;
import frc.robot.subsystems.Neck;

public class NeckStable extends Command {
  /** Creates a new NeckStable. */
  Neck m_Neck;

  public NeckStable(Neck theNeck) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Neck = theNeck;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!(m_Neck.getNeckAngle() > NeckConstants.KEncoderDeadbandThreshold))
    {
      m_Neck.getMotorController().set(NeckConstants.kNeckForwardSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NeckConstants;
import frc.robot.subsystems.Neck;

public class NeckStable extends Command {
  /** Creates a new NeckStable. */
  Neck m_Neck;
  Rotation2d m_target;

  public NeckStable(Neck theNeck) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Neck = theNeck;
    addRequirements(m_Neck);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Use the current angle as the target

    m_target = Rotation2d.fromDegrees(m_Neck.getNeckAngle());
    SmartDashboard.putNumber("Neck Stable Target Radians", m_target.getRadians()); 
    SmartDashboard.putNumber("Neck Stable Target Degrees", m_target.getDegrees()); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_target.getDegrees() > NeckConstants.KEncoderDeadbandThreshold)
    {
      m_Neck.moveTo(m_target);
      SmartDashboard.putString("RunningArm:", "Stabilizing");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_Neck.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NeckConstants;
import frc.robot.subsystems.Neck;
import frc.robot.RobotContainer;

public class SetNeckAngle extends Command {
  // Get rid of later
  /** Creates a new SetNeckAngle. */
  private final Neck m_Neck;
  Rotation2d m_target;
  private double m_neckAngle;

  public SetNeckAngle(Neck Neck, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_neckAngle = angle;
    m_Neck = Neck;
    addRequirements(m_Neck);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Insert the get distance from speaker than this subsystem should set neck angle
    SmartDashboard.putNumber("Target Neck Angle", m_neckAngle); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((m_neckAngle > NeckConstants.KEncoderDeadbandThreshold)
    && (m_neckAngle < NeckConstants.kEncoderUpperThreshold))
    {
      Rotation2d target = Rotation2d.fromRadians(m_neckAngle);
      m_Neck.moveToAngle(target);
      SmartDashboard.putString("RunningNeck:", "MovingToAngle " + m_neckAngle);
    }
    else
    {
      m_Neck.stop();
    }
  }

    
  

  
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Neck.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Neck.getNeckAngle() > m_neckAngle;
  }


}

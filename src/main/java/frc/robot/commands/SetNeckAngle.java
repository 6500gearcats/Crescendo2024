// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NeckConstants;
import frc.robot.commands.MoveNeckDown;
import frc.robot.commands.MoveNeckUp;
import frc.robot.subsystems.Neck;
import frc.robot.Constants;

public class SetNeckAngle extends Command {
  // Get rid of later
  private static final double kDefaultNeckSetAngle = 1;
  /** Creates a new SetNeckAngle. */
  private final Neck m_Neck;
  Rotation2d m_target;
  private double m_angle;

  public SetNeckAngle(Neck Neck, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_angle = angle;
    m_Neck = Neck;
    addRequirements(m_Neck);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Insert the get distance from speaker than this subsystem should set neck angle
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_angle > NeckConstants.KEncoderDeadbandThreshold)
    {
      m_Neck.moveTo(m_target);
      SmartDashboard.putString("RunningNeck:", "MovingToAngle");
    }
    /*
    else
    {
      m_Neck.stop();
    }
    */
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

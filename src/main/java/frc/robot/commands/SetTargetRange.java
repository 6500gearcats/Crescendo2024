// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NeckConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Neck;

public class SetTargetRange extends Command {

  public final Navigation m_Navigation;
  public final Neck m_Neck;
  
  private double m_targetAngle = 0.0;
  private double m_absoluteDifference = 0.0;

  /** Creates a new SetTargetRange. */
  public SetTargetRange(Navigation nav, Neck neck) {
    m_Navigation = nav;
    m_Neck = neck;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(nav, neck);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double distance = m_Navigation.getRange();
    m_targetAngle = distance * ShooterConstants.kShooterDistanceFactor;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        
    double difference = m_Neck.getNeckAngle() - m_targetAngle;
    m_absoluteDifference = Math.abs(difference);
    if(m_absoluteDifference < ShooterConstants.kRangeAngleError)
    {
      if(difference < m_targetAngle)
        {
          m_Neck.move(NeckConstants.kNeckForwardSpeed);
        }
    else if(difference > m_targetAngle)
        {
          m_Neck.move(NeckConstants.kNeckReverseSpeed);
        }
        else this.cancel();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_absoluteDifference < ShooterConstants.kRangeAngleError);
  }
}
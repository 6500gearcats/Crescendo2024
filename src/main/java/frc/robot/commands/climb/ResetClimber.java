// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ResetClimber extends Command {
  //Instance variable for climber subsystem
  Climber m_climber;
  
  /** Creates a new ResetClimber. */
  public ResetClimber(Climber theClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = theClimber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setClimberSpeed(ClimberConstants.kClimberSpeedDown);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.ArmIsFullyStowed();
    //return false;
  }
}


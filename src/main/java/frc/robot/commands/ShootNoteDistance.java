// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.NeckConstants;
import frc.robot.Constants.ShootNoteConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShootNoteDistance extends Command {
  
  private final Shooter m_ShooterSystem;
  private final Intake m_IntakeSystem;
  private long seconds;

  public ShootNoteDistance(Shooter theShooter, Intake theIntake) {
    m_ShooterSystem = theShooter;
    m_IntakeSystem = theIntake;
    addRequirements(m_ShooterSystem);
    addRequirements(m_IntakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSystem.setDistanceShooterSpeedFast();
    seconds = System.currentTimeMillis();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSystem.setDistanceShooterSpeedFast();
    if (m_ShooterSystem.distanceShooterSpeedSetFast()){
      m_IntakeSystem.setFeedSpeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSystem.stopShooter();
    m_IntakeSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_IntakeSystem.NoteIsPresent();
    //return false;
  }


  }


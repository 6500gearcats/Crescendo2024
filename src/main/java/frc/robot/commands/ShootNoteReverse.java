// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShootNoteConstants;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShootNoteReverse extends Command {
  
  private final Shooter m_ShooterSystem;
  private final Intake m_IntakeSystem;
  private long seconds;

  public ShootNoteReverse(Shooter theShooter, Intake theIntake) {
    m_ShooterSystem = theShooter;
    m_IntakeSystem = theIntake;
    addRequirements(m_ShooterSystem);
    addRequirements(m_IntakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSystem.setShooterSpeedFastReverse();
    seconds = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSystem.setShooterSpeedFastReverse();
    if (m_ShooterSystem.shooterSpeedSetFast()){
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
    //return m_IntakeSystem.NoteIsPresent();
    return false;
  }

  
  private boolean secondPast() {
    long currentSeconds = System.currentTimeMillis();

    return (currentSeconds - seconds) >= ShootNoteConstants.kmiliSeconds; 
  }

  //For sensor, not added yet
  private boolean sensorPast() {
    return false;
  }

  }


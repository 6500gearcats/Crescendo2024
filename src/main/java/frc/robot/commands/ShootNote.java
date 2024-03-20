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

public class ShootNote extends Command {
  
  private final Shooter m_ShooterSystem;
  private final Intake m_IntakeSystem;
  private long seconds;
  private double ShooterSpeed;
  private double speedWant;


  public ShootNote(Shooter theShooter, Intake theIntake) {
    m_ShooterSystem = theShooter;
    m_IntakeSystem = theIntake;
    ShooterSpeed = ShooterConstants.kShooterSpeedFast;
    speedWant = ShooterConstants.kShooterFastRPM;
    addRequirements(m_ShooterSystem);
    addRequirements(m_IntakeSystem);
  }

  public ShootNote(Shooter theShooter, Intake theIntake, double Speed, double speedWant) {
    m_ShooterSystem = theShooter;
    m_IntakeSystem = theIntake;
    ShooterSpeed = Speed;
    this.speedWant = speedWant;
    addRequirements(m_ShooterSystem);
    addRequirements(m_IntakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSystem.setShooterSpeed(ShooterSpeed);
    seconds = System.currentTimeMillis();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSystem.setShooterSpeed(ShooterSpeed);
    if (m_ShooterSystem.shooterSpeedSet(speedWant)){
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


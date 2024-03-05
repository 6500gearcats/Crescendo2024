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
  public final Navigation m_Navigation;
  public final Neck m_Neck;
  private long seconds;

  public ShootNote(Shooter theShooter, Intake theIntake, Navigation theNav, Neck theNeck) {
    m_ShooterSystem = theShooter;
    m_IntakeSystem = theIntake;
    m_Navigation = theNav;
    m_Neck = theNeck;
    addRequirements(m_ShooterSystem);
    addRequirements(m_IntakeSystem);
    addRequirements(m_Navigation);
    addRequirements(m_Neck);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSystem.setShooterSpeedFast();
    seconds = System.currentTimeMillis();
    
  }

  public boolean setNeckAngle() {
    //Get distance from April Tag (or gyro) --> get "range" from getRange() in Navigation 
    //Slope = 12.391 --> kShooterDistanceFactor
    double distance = m_Navigation.getRange();
    double targetAngle = distance * ShooterConstants.kShooterDistanceFactor;
    //m_Neck.getMotorController().set(targetAngle);
    double encoderDifference = m_Neck.getNeckAngle() - targetAngle;
    if(encoderDifference != targetAngle)
    {
      if(encoderDifference < targetAngle)
        {
          m_Neck.move(NeckConstants.kNeckForwardSpeed);
        }
    else if(encoderDifference > targetAngle)
        {
          m_Neck.move(NeckConstants.kNeckReverseSpeed);
        }
    else if(m_Neck.getNeckAngle() > 0.1)
        {
          this.cancel();
        }
    }
    else{
      return true;
    }
    return false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(setNeckAngle()){
    m_ShooterSystem.setShooterSpeedFast();
    if (m_ShooterSystem.shooterSpeedSetFast()){
      m_IntakeSystem.setFeedSpeed();
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSystem.stopShooter();
    m_IntakeSystem.stop();
    m_Neck.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IntakeSystem.NoteIsPresent();
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


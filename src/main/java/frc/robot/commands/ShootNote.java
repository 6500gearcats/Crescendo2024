// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
  
  private final Shooter m_ShooterSystem;
  private final Intake m_IntakeSystem;

  public ShootNote(Shooter theShooter, Intake theIntake) {
    m_ShooterSystem = theShooter;
    m_IntakeSystem = theIntake;
    addRequirements(m_ShooterSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSystem.setShooterSpeedFast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ShooterSystem.shooterSpeedSetFast()){
      m_IntakeSystem.setPickupSpeed();
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class BackwardsIntake extends Command {
  private final Shooter m_ShooterSystem;
  private final Intake m_IntakeSystem;
  /** Creates a new BackwardsIntake. */
  public BackwardsIntake(Shooter theShooter, Intake theIntake) {
    m_ShooterSystem = theShooter;
    m_IntakeSystem = theIntake;
    addRequirements(m_ShooterSystem);
    addRequirements(m_IntakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSystem.setReverseSpeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.stop();
  }
}

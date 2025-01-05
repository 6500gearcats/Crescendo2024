// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class BackwardsIntake extends Command {
  private final Intake m_IntakeSystem;
  private final Shooter m_ShooterSystem;
  /** Creates a new BackwardsIntake. */
  public BackwardsIntake(Intake theIntake, Shooter theShooter) {
    m_IntakeSystem = theIntake;
    m_ShooterSystem = theShooter;
    addRequirements(m_IntakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSystem.setReverseSpeed();
    m_ShooterSystem.reverseShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.stop();
    m_ShooterSystem.stopShooter();
  }
}

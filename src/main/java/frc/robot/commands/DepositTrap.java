// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class DepositTrap extends Command {
  /** Creates a new DepositTrap. */
  Shooter m_robotShooter;
  Intake m_Intake;

  public DepositTrap(Shooter theShooter, Intake theIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotShooter = theShooter;
    m_Intake = theIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.setPickupSpeed();
    m_robotShooter.reverseMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stop();
    m_robotShooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

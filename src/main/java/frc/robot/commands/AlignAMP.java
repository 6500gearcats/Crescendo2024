// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Navigation;
import frc.robot.Vision;
import frc.robot.subsystems.DriveSubsystem;


public class AlignAMP extends Command {
  /** Creates a new AlignAMP. */
  Vision m_vision;
  DriveSubsystem m_DriveSubsystem;

  public AlignAMP(Vision m_Vision, DriveSubsystem m_driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = m_Vision;
    m_DriveSubsystem = m_driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xTransform;
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Red) {
          xTransform = m_vision.getChosenTargetRotation(12);
    }
    else {
       xTransform = m_vision.getChosenTargetRotation(6);
    }

    if (xTransform != 0) {
      m_DriveSubsystem.drive(-xTransform, -xTransform, -xTransform, false);
    }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(0,0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

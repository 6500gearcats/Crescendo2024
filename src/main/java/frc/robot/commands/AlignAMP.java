// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Navigation;
import frc.robot.Vision;
import frc.robot.subsystems.DriveSubsystem;


public class AlignAMP extends Command {
  /** Creates a new AlignAMP. */
  double yTransform;
  Vision m_vision;
  DriveSubsystem m_DriveSubsystem;

  public AlignAMP(Vision m_Vision, DriveSubsystem m_driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = m_Vision;
    m_DriveSubsystem = m_driveSubsystem;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Red) {
      yTransform = m_vision.getChosenTargetRotation(5);
    }
    else {
      yTransform = m_vision.getChosenTargetRotation(6);
    }

    if (yTransform != 0) {
      m_DriveSubsystem.drive(0, yTransform * 0.3, 0, false);
    }

    SmartDashboard.putNumber("Y-transform", yTransform);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(yTransform) < 0.05 || yTransform == 0;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

public class GetChosenTarget extends Command {
  /** Creates a new GetBestTarget. */
private final Navigation m_vision;
private final DriveSubsystem m_drive;
private final int targetID;

  public GetChosenTarget(Navigation vision, DriveSubsystem drive, int chosenID) {
    m_drive = drive;
    m_vision = vision;
    targetID = chosenID;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = m_vision.getChosenTargetRotation(targetID);
    double speed = m_vision.getChosenTargetRange(targetID);
    if(speed > 0.5)
    {
      speed = 0.5;
    }
    else if (speed < -0.5) {
      speed = -0.5;    
    }

    SmartDashboard.putNumber("Sim-Robot (Vision) Chosen Target Speed", speed);
    SmartDashboard.putNumber("Sim-Robot (Vision) Chosen Target Rotation", rotation);
    if(rotation != 0)
    {
      m_drive.drive(speed, 0, rotation, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteFinder;

public class MoveToClosestNote extends Command {
  /** Creates a new MoveToClosestNote. */
  private final NoteFinder m_NoteFinder;
  private final DriveSubsystem m_drive;
  private final Intake m_IntakeSystem;

  public MoveToClosestNote(NoteFinder finder, DriveSubsystem drive, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_NoteFinder = finder;
    m_IntakeSystem = intake;
    ;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = m_NoteFinder.getRotation(); 
    double speed = m_NoteFinder.getRange();

    

    if(speed > 2)
    {
      speed = 2;
    }
    else if (speed < -2) {
      speed = -2;    
    }
    speed = Math.abs(speed);


    if(Math.abs(rotation)  > 0.05)
    {
      m_drive.drive(speed, 0, -rotation, false);
    }
    else if(speed != 0){
      m_drive.drive(speed, 0, -rotation, false);
    }
    else {
      m_drive.drive(Constants.kDefaultNoteFinderSpeed, 0, 0, false);
    } 
    
       

        
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
}

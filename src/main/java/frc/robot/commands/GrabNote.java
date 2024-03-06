// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteFinder;

public class GrabNote extends ParallelRaceGroup {
  /** Creates a new Parallelgroup. */
  public GrabNote(NoteFinder finder, DriveSubsystem drive, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new MoveToClosestNote(finder, drive, intake),
      new PickUpNote(intake)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpNearestNote extends SequentialCommandGroup {
  private int posX;
  private int posY;
  private Intake m_intakeSystem;
  private DriveSubsystem m_drive;
  public PickUpNearestNote(Intake theIntake, DriveSubsystem drive) {
    m_intakeSystem = theIntake;
    m_drive = drive;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    // Use Vision in NoteFinder to pass Note Position (Into posX and posY)
    new PathFindToPos(posX, posY, m_drive),
    new PickUpNote(m_intakeSystem)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.LowerHooks;
import frc.robot.commands.climb.RaiseHooks;
import frc.robot.commands.climb.ResetClimber;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullTrapSequence extends SequentialCommandGroup {
  /** Creates a new FullTrapSequence. */
  public FullTrapSequence(Shooter m_shooter, Intake m_intake, Climber m_climber, Neck m_Neck, DriveSubsystem m_drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Assumes robot is centered in position with climbers up against the stage wall
      //new InstantCommand(() -> m_drive.removeDefaultCommand()),
      //new InstantCommand(() -> m_drive.setDriveCoast()),
      new ParallelDeadlineGroup(
      new MoveNeckUp(m_Neck),
      new RunCommand(() -> m_drive.drive(0.1,0,0,false)).withTimeout(3.0)
      ),
      // Step 1:
      // Step 2:
      new RunCommand(() -> m_drive.drive(0.2, 0, 0, false)).withTimeout(.4),
      // Step 3:
      new ResetClimber(m_climber),
      // Step 4:
      new ParallelDeadlineGroup(
      new ShootWhileStable(m_climber, m_shooter, m_intake),
      new MoveNeckUp(m_Neck)
      ),
      new RaiseHooks(m_climber).withTimeout(.5)
    );
  }
}

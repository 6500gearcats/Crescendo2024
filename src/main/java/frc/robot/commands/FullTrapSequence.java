// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.RaiseHooks;
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
  public FullTrapSequence(Shooter m_shooter, Intake m_intake, Climber m_climber, Neck m_Neck, DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Assumes robot is in position with climbers up against the stage wall

      // Step 1: 
      new MoveNeckUp(m_Neck),
      // Step 2:
      new RaiseHooks(m_climber),
      // Step 3:
      new ShootNoteReverse(m_shooter, m_intake)
    );
  }
}

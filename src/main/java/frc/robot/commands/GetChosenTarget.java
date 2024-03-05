package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

public class GetChosenTarget extends Command {
  /** Creates a new GetBestTarget. */
private final Vision m_vision;
private final DriveSubsystem m_drive;
private SendableChooser<Integer> m_chooser;

final double ANGULAR_P = 0.1;
final double ANGULAR_D = 0.0;
PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  public GetChosenTarget(Vision vision, DriveSubsystem drive) {
    m_drive = drive;
    m_vision = vision;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int targetID = 5; //m_chooser.getSelected().intValue();
    double rotation = m_vision.getChosenTargetRotation(targetID);
    double speed = m_vision.getChosenTargetRange(targetID);
    rotation = -turnController.calculate(rotation, 0) * Constants.kRangeSpeedOffset;
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
  public void end(boolean interrupted) {
    Commands.print("Command ended = " + interrupted);
  }

  // Returns true when the command should end.
  //@Override
  /*public boolean isFinished() {
    return;
  }*/
  
}
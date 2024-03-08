package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.apriltag.AprilTag;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;
import frc.robot.utility.FieldTags;

public class MoveToAmp extends Command {
  /** Creates a new GetBestTarget. */
private final Vision m_vision;
private final DriveSubsystem m_drive;

private int m_target;

final double ANGULAR_P = 0.1;
final double ANGULAR_D = 0.0;
PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  public MoveToAmp(Vision vision, DriveSubsystem drive) {
    m_drive = drive;
    m_vision = vision;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    m_target = 0;
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        m_target = FieldTags.blueAmp;
      }
      else 
      if (alliance.get() == Alliance.Red) {
        m_target = FieldTags.redAmp;
      }

      if (!m_vision.isVisible(m_target)){
        this.cancel();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = m_vision.getChosenTargetRotation(m_target);
    double range = m_vision.getChosenTargetRange(m_target);
    rotation = -turnController.calculate(rotation, 0) * Constants.kRangeSpeedOffset;
    SmartDashboard.putNumber("Sim-Robot (Vision) Chosen Target Speed", range);
    SmartDashboard.putNumber("Sim-Robot (Vision) Chosen Target Rotation", rotation);
    if(rotation != 0)
    {
      m_drive.drive( DriveConstants.kNormalSpeedMetersPerSecond, rotation, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Commands.print("Command ended = " + interrupted);
  }

  // Returns true when the command should end.
  //@Override
  public boolean isFinished() {
    double range = m_vision.getChosenTargetRange(m_target);
    return range < 1;
  }
  
}
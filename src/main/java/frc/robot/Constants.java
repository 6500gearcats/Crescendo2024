// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.CANSparkBase.IdleMode;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3.5; //3
    public static final double kNormalSpeedMetersPerSecond = 0.85; //1.5
    public static final double kMaxAngularSpeed = 0.75 * Math.PI; // radians per second (was 2)

    //turbo
    public static final double kTurboModeModifier = 7.0/2.0;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(28.0);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.0);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kGunnerControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

// These constants are not correct, temporary until parts added.
  
  public static final class ArmConstants {
    // These are port values (where it's located on the robot)
    public static final int kArmMotorPort = 10;  
    // These are speed values
    public static final double kArmReverseSpeed = 0.6;
    public static final double kArmForwardSpeed = -0.7;
    public static final double kArmForwardMaxSpeed = 0.9;
    public static final double kArmReverseMaxSpeed = -0.9;

    // TODO tune
    public static final double kArmSlowModifier = 0.57;
    
    //not set right
    //encoder values
    public static final double kEncoderUpperThreshold = 0.30;
    public static final double kArmStowAngle = 0.8;
    public static final double kArmFloorAngle = 0;
    public static final double kArmHighAngle = 0.315; //good
    public static final double kArmMidAngle = 0.37; //good
    public static final double kArmLowAngle = 0.70;
    public static final double kLoadingStation = 0.36;
  }

  public static final class ClawConstants {
    //ports
    public static final int kClawMotorPort = 9;
    public static final int kOpen = 0;
    public static final int kClose = 0;
    //speeds (claw is reversed)
    public static final double kClawReverseSpeed = 0.75;
    public static final double kClawForwardSpeed = -0.75;
    public static final double kClawForwardMaxSpeed = -0.9;
    public static final double kClawReverseMaxSpeed = 0.9;

    // TODO tune
    public static final double kSlowClawModifier = 0.5;

    //not set right
    //encoder values
    public static final double kClawStowAngle = 0.49; //good
    public static final double kClawFloorAngle = 0;
    public static final double kClawHighAngle = 0.67;
    public static final double kClawMidAngle = 0.67; //probably same as high
    public static final double kClawLowAngle = 0.67; //probably same as high
    public static final double kClawLowerLimit = 0.27; //good
    public static final double kClawUpperLimit = 0.815; //good
    public static final double kClawTolerance = 0.01; //good
  }

  public static final class GyroConstants{
    public static final double kTiltPitch = 65; //11? tilt angle=
    public static final double kPlatformLevel = 7;
  }

  public static final class PlatformConstants {
    public static final double kPlatformSpeed = 0.4;
  }


  //Vision consants
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kFarTgtXPos = 0.1651;
    public static final double kFarTgtYPos = 0.1651;
    public static final double kFarTgtZPos = 0.1651;
  
    public static final Pose3d kFarTargetPose = 
        new Pose3d(
            new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
            new Rotation3d(0.0, 0.0, 3.14159)
        );
    public static final double kCameraToRobot = 0.1; //?
  }

  //PID Controller constants
  public static final double ANGULAR_P = 0.1;
  public static final double ANGULAR_D = 0.0;
  public static final double P_GAIN = 0.1;
  public static final double D_GAIN = 0.0;

  //Camera constants
  public static final String kCamName = "photonvision";

}

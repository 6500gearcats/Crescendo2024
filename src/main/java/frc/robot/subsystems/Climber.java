// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;

import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.AbsoluteEncoder;




import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.NeckConstants;
import frc.robot.utility.EncoderOdometer;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public final SparkMax m_LeftClimberMotor = new SparkMax(ClimberConstants.kLeft_ClimberMotorPort,SparkLowLevel.MotorType.kBrushless);
  public final SparkMax m_RightClimberMotor = new SparkMax(ClimberConstants.kRight_ClimberMotorPort,SparkLowLevel.MotorType.kBrushless);
  private final DigitalInput m_LeftArmDownSensor = new DigitalInput(0);
  private final DigitalInput m_RightArmDownSensor = new DigitalInput(1);
  
  private final AbsoluteEncoder m_leftClimberEncoder;
  private final AbsoluteEncoder m_rightClimberEncoder;

    //private RelativeEncoder m_winchEncoder;
  private RelativeEncoder m_winchEncoder;
  private EncoderOdometer m_winchOdometer;

  private SparkClosedLoopController leftPIDcontroller;
  private SparkClosedLoopController rightPIDcontroller;
  private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(NeckConstants.kNeck_kS, NeckConstants.kNeck_kG, NeckConstants.kNeck_kV);
  
  public Climber() {

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
      .inverted(true);
    rightConfig.closedLoop
      .pid(.5,NeckConstants.kNeck_kI,NeckConstants.kNeck_kD);

    m_RightClimberMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.closedLoop
      .pid(.5,NeckConstants.kNeck_kI,NeckConstants.kNeck_kD);
    m_LeftClimberMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // int m_lowerLimit = m_LeftClimberMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    // int m_upperLimit = m_LeftClimberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_leftClimberEncoder = m_LeftClimberMotor.getAbsoluteEncoder();
    m_rightClimberEncoder = m_RightClimberMotor.getAbsoluteEncoder();

    m_winchEncoder = m_LeftClimberMotor.getEncoder();
    m_winchOdometer = new EncoderOdometer(m_winchEncoder);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Left Hook Down", !m_LeftArmDownSensor.get());
    SmartDashboard.putBoolean("Right Hook Down", !m_RightArmDownSensor.get());
    SmartDashboard.putNumber("Left arm encoder", m_leftClimberEncoder.getPosition());
    SmartDashboard.putNumber("Right arm encoder", m_rightClimberEncoder.getPosition());
    SmartDashboard.putNumber("Arm position", m_winchOdometer.getPosition());
  }

  public void setClimberSpeed() {
    m_LeftClimberMotor.set(ClimberConstants.kClimberSpeed);
    m_RightClimberMotor.set(ClimberConstants.kClimberSpeed);
  }

  public void setClimberSpeed(double speed) {
    m_LeftClimberMotor.set(speed);
    m_RightClimberMotor.set(speed);
  }

  public void stop(){
    m_LeftClimberMotor.stopMotor();
    m_RightClimberMotor.stopMotor();
  }

 /*  public boolean ArmIsFullyExtended() {
    // boolean lowerLimit = m_lowerLimit.isPressed();
    // boolean upperLimit = m_upperLimit.isPressed();
    // SmartDashboard.putBoolean("Upper limit", upperLimit);
    // SmartDashboard.putBoolean("Lower limit", lowerLimit);
    boolean isArmExtended = !m_armSensor.get();
    // Use ColorSensor to determine if true
    return isArmExtended;
  }
  */

  public boolean ArmIsFullyExtended() {
    if(m_winchOdometer.getPosition() > ClimberConstants.kMaxArmHeight)  
    {
      return true;
    }
    return false;
  }

  public double getArmHeights() {
    return m_winchOdometer.getPosition();
  }

  public double getArmHeightsEncoder() {
    return m_winchEncoder.getPosition();
  }

  public boolean ArmIsFullyStowed() {
    if((m_LeftArmDownSensor.get()) || 
    (m_RightArmDownSensor.get())) 
      {
        return false;
      }
    m_winchOdometer.reset();
    return true;
  }

  public void moveTo(double target) {
  leftPIDcontroller.setReference(
                target,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0);
  rightPIDcontroller.setReference(
                target,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0);
  }
}
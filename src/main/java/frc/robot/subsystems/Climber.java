// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

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
  public final CANSparkMax m_LeftClimberMotor = new CANSparkMax(ClimberConstants.kLeft_ClimberMotorPort,CANSparkLowLevel.MotorType.kBrushless);
  public final CANSparkMax m_RightClimberMotor = new CANSparkMax(ClimberConstants.kRight_ClimberMotorPort,CANSparkLowLevel.MotorType.kBrushless);
  private final DigitalInput m_LeftArmDownSensor = new DigitalInput(0);
  private final DigitalInput m_RightArmDownSensor = new DigitalInput(1);
  
  private final AbsoluteEncoder m_leftClimberEncoder;
  private final AbsoluteEncoder m_rightClimberEncoder;

    //private RelativeEncoder m_winchEncoder;
  private RelativeEncoder m_winchEncoder;
  private EncoderOdometer m_winchOdometer;

  private SparkPIDController leftPIDcontroller;
  private SparkPIDController rightPIDcontroller;
  private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(NeckConstants.kNeck_kS, NeckConstants.kNeck_kG, NeckConstants.kNeck_kV);
  
  public Climber() {

    m_RightClimberMotor.setInverted(true);
    // int m_lowerLimit = m_LeftClimberMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    // int m_upperLimit = m_LeftClimberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_leftClimberEncoder = m_LeftClimberMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_rightClimberEncoder = m_RightClimberMotor.getAbsoluteEncoder(Type.kDutyCycle);

    m_winchEncoder = m_LeftClimberMotor.getEncoder();
    m_winchOdometer = new EncoderOdometer(m_winchEncoder);

    leftPIDcontroller = m_LeftClimberMotor.getPIDController();
    leftPIDcontroller.setP(.5);
    leftPIDcontroller.setI(NeckConstants.kNeck_kI);
    leftPIDcontroller.setD(NeckConstants.kNeck_kD);

    rightPIDcontroller = m_RightClimberMotor.getPIDController();
    rightPIDcontroller.setP(.5);
    rightPIDcontroller.setI(NeckConstants.kNeck_kI);
    rightPIDcontroller.setD(NeckConstants.kNeck_kD);
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
                0);
  rightPIDcontroller.setReference(
                target,
                ControlType.kPosition,
                0);
  }
}
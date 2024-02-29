// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;



public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public final CANSparkMax m_LeftClimberMotor = new CANSparkMax(ClimberConstants.kLeft_ClimberMotorPort,MotorType.kBrushless);
  public final CANSparkMax m_RightClimberMotor = new CANSparkMax(ClimberConstants.kRight_ClimberMotorPort,MotorType.kBrushless);
  private final DigitalInput m_LeftArmDownSensor = new DigitalInput(0);
  private final DigitalInput m_RightArmDownSensor = new DigitalInput(1);
  
  private final AbsoluteEncoder m_leftClimberEncoder;
  private final AbsoluteEncoder m_rightClimberEncoder;

  
  public Climber() {
    // int m_lowerLimit = m_LeftClimberMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    // int m_upperLimit = m_LeftClimberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_leftClimberEncoder = m_LeftClimberMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_rightClimberEncoder = m_RightClimberMotor.getAbsoluteEncoder(Type.kDutyCycle);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Left arm encoder", m_LeftArmDownSensor.get());
    SmartDashboard.putBoolean("Right arm encoder", m_RightArmDownSensor.get());

  }

  public void setClimberSpeed() {
    m_LeftClimberMotor.set(ClimberConstants.kClimberSpeed);
    m_RightClimberMotor.set(ClimberConstants.kClimberSpeed);
  }

  public void setClimberSpeed(double speed) {
    m_LeftClimberMotor.set(speed);
    m_RightClimberMotor.set(-speed);
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
    if((m_leftClimberEncoder.getPosition() > ClimberConstants.kMaxArmHeight) || 
    (m_rightClimberEncoder.getPosition() > ClimberConstants.kMaxArmHeight)) 
      {
        return true;
      }
    return false;
  }

  public boolean ArmIsFullyStowed() {
    if((m_LeftArmDownSensor.get()) || 
    (m_RightArmDownSensor.get())) 
      {
        return false;
      }
    return true;
  }

}
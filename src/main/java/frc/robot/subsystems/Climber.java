// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;



public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public final CANSparkMax m_LeftClimberMotor = new CANSparkMax(ClimberConstants.kLeft_ClimberMotorPort,MotorType.kBrushless);
  public final CANSparkMax m_RightClimberMotor = new CANSparkMax(ClimberConstants.kRight_ClimberMotorPort,MotorType.kBrushless);
  
  SparkLimitSwitch m_lowerLimit;
  SparkLimitSwitch m_upperLimit;

  public Climber() {
    m_lowerLimit = m_LeftClimberMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_upperLimit = m_LeftClimberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  

  public boolean ArmIsFullyExtended() {
    boolean lowerLimit = m_lowerLimit.isPressed();
    boolean upperLimit = m_upperLimit.isPressed();
    SmartDashboard.putBoolean("Upper limit", upperLimit);
    SmartDashboard.putBoolean("Lower limit", lowerLimit);
    return true;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utility.EncoderOdometer;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {
    public final CANSparkMax kRight_ClimberMotor = new CANSparkMax(ClimberConstants.kRight_ClimberMotorPort,MotorType.brushless);
    m_lowerLimit = m_winchMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_upperLimit = m_winchMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
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
}

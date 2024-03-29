// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax m_sparkMax = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);

  public final MotorController m_ShooterMotor = m_sparkMax;
  private boolean m_bNoteFired = false; 
  private boolean m_bShooterAtSpeed = false;

  private final RelativeEncoder m_shooterEncoder = m_sparkMax.getEncoder();
  
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Motor rotation", m_shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Motor speed", shooterSpeed());

  }

  public double shooterSpeed() {
    return m_ShooterMotor.get();

  }

  public void setShooterSpeedFast(){
    m_ShooterMotor.set(ShooterConstants.kShooterSpeedFast);
  }

  public void setDistanceShooterSpeedFast(){
    m_ShooterMotor.set(ShooterConstants.kDistanceShooterSpeedFast);
  }

  public void setShooterSpeedFastReverse() {
    m_ShooterMotor.set(ShooterConstants.kShooterReverseFast);
  }

  public boolean shooterSpeedSetFast(){
    m_bShooterAtSpeed = Math.abs(m_shooterEncoder.getVelocity()) >= ShooterConstants.kShooterFastRPM;
    return m_bShooterAtSpeed;
  }

  public boolean distanceShooterSpeedSetFast(){
    m_bShooterAtSpeed = Math.abs(m_shooterEncoder.getVelocity()) >= ShooterConstants.kDistanceShooterRPM;
    return m_bShooterAtSpeed;
  }

  public boolean trapSpeedSetFast(){
    m_bShooterAtSpeed = Math.abs(m_shooterEncoder.getVelocity()) >= ShooterConstants.kShooterTrapRPM;
    return m_bShooterAtSpeed;
  }

  public void stopShooter() {
    m_ShooterMotor.stopMotor();
  }
  
  public void reverseShooter() {
    m_ShooterMotor.set(ShooterConstants.kBackwardsShooter);
  }

  public void reverseMotor() {
    m_ShooterMotor.set(-ShooterConstants.kShooterSpeedSlow);
  }

  public void reverseMotorFast() {
    m_ShooterMotor.set(-ShooterConstants.kShooterSpeedSlow);
  }

}

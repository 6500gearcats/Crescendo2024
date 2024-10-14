// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  //private final DigitalInput m_noteSensor = new DigitalInput(9);
  private final DigitalInput m_noteSwitch = new DigitalInput(8); //TODO: Put this on correct channel
  /** Creates a new Intake. */
public final MotorController m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("Note Sensor", NoteInPlace());
    SmartDashboard.putBoolean("Note Switch", NoteIsPresent());

  }
   public void setPickupSpeed() {
    m_intakeMotor.set(IntakeConstants.kPickUpSpeed);
  }

  public void setPickupSpeedSlow() {
    m_intakeMotor.set(IntakeConstants.kPickUpSpeedSlow);
  }

  public boolean NoteIsPresent() {
    boolean ringIsPresent = !m_noteSwitch.get();
    // Uses switches on sides to determine if true
    return ringIsPresent;
  }

  //public boolean NoteInPlace() {
  //  boolean ringInPlace = !m_noteSensor.get();
    // Uses vision sensor to determine if true
  //  return ringInPlace;
  //}

  public void stop() {
    m_intakeMotor.stopMotor();
  }

  public void setFeedSpeed() {
    m_intakeMotor.set(IntakeConstants.kFeedSpeed);
  }

  public void setReverseSpeed() {
    m_intakeMotor.set(IntakeConstants.kReverseFeedSpeed);
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
public final MotorController m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushed);

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   public void setPickupSpeed() {
    //m_intakeMotor.set(abc); - Set motor to whatever constant we'll define later
  }

  public boolean NoteIsPresent() {
    boolean ballIsPresent = false;
    // Use ColorSensor to determine if true
    return ballIsPresent;
  }

  public void stop() {
    m_intakeMotor.stopMotor();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.NeckConstants;

public class NeckPID extends PIDSubsystem {

    private final CANSparkMax m_CanSparkMaxNeck = new CANSparkMax(NeckConstants.kNeckMotorPort, MotorType.kBrushless); 
    private final MotorController m_neckMotor =  m_CanSparkMaxNeck;

    private final AbsoluteEncoder m_neckEncoder;

  
  /** Creates a new NeckPID. */
  public NeckPID() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
        m_neckEncoder = m_CanSparkMaxNeck.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_neckEncoder.getPosition();
  }
}

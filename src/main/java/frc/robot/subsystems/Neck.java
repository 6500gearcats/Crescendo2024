// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//EDIT PORTS; create code!
package frc.robot.subsystems;

import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants.NeckConstants;

public class Neck extends SubsystemBase {
  /** Creates a new Neck. */
  public boolean slowNeck;

    // Create the Neck tilter motor and claw tilter motor
    // The constants are not corect right now, will be replaced.

    private final CANSparkMax m_CanSparkMaxNeck = new CANSparkMax(NeckConstants.kNeckMotorPort, MotorType.kBrushless); 
    private final MotorController m_neckMotor =  m_CanSparkMaxNeck;

    private final AbsoluteEncoder m_neckEncoder;

    // Sets upper and lower limit
    //private final DutyCycleEncoder m_tiltNeckEncoder = new DutyCycleEncoder(0);

    // Sets lower limit
    //private SparkMaxLimitSwitch m_lowerLimitSwitch; 
    //private final DigitalInput m_lowerLimitSwitch = new DigitalInput(2);

    //private boolean m_isNeckStored = false;
    //private boolean m_isNeckCapped = false;

    private final SlewRateLimiter NeckFilter = new SlewRateLimiter(0.6);

    private double NeckPosition;
    //private boolean lowerLimit;

  public Neck() {
    m_neckEncoder = m_CanSparkMaxNeck.getAbsoluteEncoder(Type.kDutyCycle);

    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(m_CanSparkMaxNeck, DCMotor.getNEO(1)); }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NeckPosition = m_neckEncoder.getPosition();
    //lowerLimit = m_lowerLimitSwitch.get();
  
    SmartDashboard.putNumber("Neck Encoder:", NeckPosition);
    SmartDashboard.putNumber("Neck motor speed", m_neckMotor.get());
    //SmartDashboard.putBoolean("Neck limit: ", m_lowerLimitSwitch.get());
  }
//Moves Neck up at constant speed
/*
public void NeckUp() {
  if ( AtMaxHeight() ) {
      m_neckMotor.set(0);
  } else {
      m_neckMotor.set(NeckConstants.kNeckForwardSpeed);
  }
}
*/

//same method that takes in a speed to be used instead of our constant, useful in the NeckUp command
/*
public void NeckUpSpeed(double speed) {
  if (slowNeck) speed *= NeckConstants.kNeckSlowModifier;
  if ( AtMaxHeight() ) {
      m_neckMotor.set(0);
  } else {
      m_neckMotor.set(NeckFilter.calculate(speed));
  }
}
*/

//Moves Neck down at constant speed
/*
public void NeckDown() {
  if (m_neckEncoder.getPosition() >= 0.75){
    m_neckMotor.set(NeckConstants.kNeckReverseSpeed*NeckConstants.kNeckSlowModifier);
  }
  else {
      m_neckMotor.set(NeckConstants.kNeckReverseSpeed);
  }
}
*/
//same method that takes in a speed to be used instead of our constant, useful in the NeckDown command
/*
public void NeckDownSpeed(double speed) {
  if (slowNeck) speed *= NeckConstants.kNeckSlowModifier;
  
}
*/

public boolean AtMaxHeight() {
  return m_neckEncoder.getPosition() > NeckConstants.kEncoderUpperThreshold;
}

public boolean AtMinHeight() {
  return m_neckEncoder.getPosition() < NeckConstants.kEncoderLowerThreshold;
}

public double getNeckAngle() {
  return m_neckEncoder.getPosition();
}

public AbsoluteEncoder getNeckEncoder()
{
  return m_neckEncoder;
}

public void stop() {
  m_neckMotor.set(0.0);
}

public void setSpeed(double speed) {
  m_neckMotor.set(speed);
}

}



//clean up; get only required/general functions

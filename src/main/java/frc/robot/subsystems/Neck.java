// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//EDIT PORTS; create code!
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.ArmFeedforward;
//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants.NeckConstants;

public class Neck extends SubsystemBase {
  /** Creates a new Neck. */
  public boolean slowNeck;

    // Create the Neck tilter motor and claw tilter motor
    // The constants are not corect right now, will be replaced.

    private final CANSparkMax m_neckMotor = new CANSparkMax(NeckConstants.kNeckMotorPort, MotorType.kBrushless); 
    //private final MotorController m_neckMotor =  m_CanSparkMaxNeck;

    private final AbsoluteEncoder m_neckEncoder;

    // Sets upper and lower limit
    //private final DutyCycleEncoder m_tiltNeckEncoder = new DutyCycleEncoder(0);

    // Sets lower limit
    //private SparkMaxLimitSwitch m_lowerLimitSwitch; 
    //private final DigitalInput m_lowerLimitSwitch = new DigitalInput(2);

    //private boolean m_isNeckStored = false;
    //private boolean m_isNeckCapped = false;

    //private final SlewRateLimiter NeckFilter = new SlewRateLimiter(0.6);
    private ArmFeedforward armFeedforward = new ArmFeedforward(NeckConstants.kNeck_kS, NeckConstants.kNeck_kG, NeckConstants.kNeck_kV);
    private double NeckPosition;
    //private boolean lowerLimit;

    private SparkPIDController neckPIDcontroller;

  public Neck() {
    m_neckEncoder = m_neckMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // See https://www.chiefdelphi.com/t/holding-up-a-wrist-with-a-neo/425787/14 to set these
    double endAngle = 0;
    double startAngle = 0;
    double valueAtEndAngle =0;


    m_neckEncoder.setPositionConversionFactor((endAngle - startAngle) / valueAtEndAngle);    

    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(m_neckMotor, DCMotor.getNEO(1)); }
    neckPIDcontroller = m_neckMotor.getPIDController();
    neckPIDcontroller.setP(NeckConstants.kNeck_kP);
    neckPIDcontroller.setI(NeckConstants.kNeck_kI);
    neckPIDcontroller.setD(NeckConstants.kNeck_kD);

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

public void move(double kneckreversespeed) {
  m_neckMotor.set(kneckreversespeed);
}

public void moveTo(Rotation2d target) {
  neckPIDcontroller.setReference(
                target.getRadians(),
                ControlType.kPosition,
                0);
}
public void moveToAngle(Rotation2d target) {
  neckPIDcontroller.setReference(
                target.getRadians(),
                ControlType.kPosition,
                0, 
                armFeedforward.calculate(target.getRadians(), 0.15));
  
}

}



//clean up; get only required/general functions

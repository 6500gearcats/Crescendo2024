// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.buttonBoxConstants;

public class Buttons extends SubsystemBase {
  /** Creates a new Buttons. */
  private EventLoop m_loop;
  private Joystick m_buttonBox;
  private ArrayList<Integer> buttonPorts = new ArrayList<Integer>();
  private ArrayList<BooleanEvent> buttons = new ArrayList<BooleanEvent>();

  public Buttons(Joystick theButtonBox) {
    m_loop = new EventLoop();
    m_buttonBox = theButtonBox;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for(int buttonPort : buttonPorts)
    {
      for(BooleanEvent button : buttons)
      {
        button = m_buttonBox.button(buttonPort, m_loop);
        SmartDashboard.putBoolean("Button " + buttonPort + ":", button.getAsBoolean());
      }
    }
  }

  public void addButton(int port)
  {
    buttonPorts.add(port);
    buttons.add(new BooleanEvent(null, null));
    
  }

  public BooleanEvent getValueAtButtonIndex(int index)
  {
    return buttons.get(index);
  }
}

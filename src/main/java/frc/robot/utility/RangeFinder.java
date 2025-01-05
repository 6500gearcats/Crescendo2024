// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class RangeFinder {
    private InterpolatingDoubleTreeMap m_map;
    public RangeFinder() {
        m_map = new InterpolatingDoubleTreeMap();
        //(distance, neck angle)
        m_map.put(-3.665,0.0887);
        m_map.put(-3.712,0.0954);
        m_map.put(-12.684,0.0400);
        m_map.put(-6.822,0.0600);
        m_map.put(-20.984,0.0400);
        m_map.put(-4.203,0.0800);
    }

    public double getNeckAngle(double distance)
    {
        return m_map.get(distance);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class RangeFinder {
    private InterpolatingDoubleTreeMap m_map;
    RangeFinder() {
        m_map.put(1.0,1.0); 
    }

    public double getNeckAngle(double distance)
    {
        return m_map.get(distance);
    }
}

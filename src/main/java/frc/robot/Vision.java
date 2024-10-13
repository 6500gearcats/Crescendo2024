/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
    private final PhotonCamera m_camera;
    // private final PhotonCamera cameraNote;
    private final PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public Vision(PhotonCamera camera) {
        m_camera = camera;

        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonPipelineResult getLatestResult() {
        return m_camera.getLatestResult();
    }

    public PhotonPipelineResult getLatestCameraResult() {
        return m_camera.getLatestResult();
    }

    public double getChosenTargetRotation(int targetID) {
        var result = getLatestCameraResult();
        // Get a list of all of the targets that have been detected.
        List<PhotonTrackedTarget> targets = result.getTargets();
        double rotation = 0;

        // For each target we have check if it matches the id you want.
        for (PhotonTrackedTarget target : targets) {
            if (result.hasTargets()) {
                if (target.getFiducialId() == targetID) {
                    // Use the value of target to find our rotation using the getYaw command
                    return target.getYaw();
                }
            } else {
                rotation = 0;
            }
        }

        return rotation;
    }

    public double getChosenTargetRange(int targetID) {
        var result = getLatestCameraResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        double range = 0;
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : targets) {
                if (target.getFiducialId() == targetID) {
                    range = PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS, // Previously declarde
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(target.getPitch()));
                    return range;
                }
            }
        }
        return 0;
    }

    public double getRange() {
        var result = getLatestResult();
        double range;
        if (result.hasTargets()) {
            // First calculate range
            range = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS, // Previously declarde
                    TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
            return range;

        } else {
            // If we have no targets, stay still.
            return 0;
            // When this is implemented - DO NOTHING IF RANGE IS 0
        }
    }

    public double getYaw() {
        var result = getLatestCameraResult();
        double yaw = 0.0;
        if (result.hasTargets()) {
            // Calculate angular turn power
            // Remove -1.0 because it was inverting results.
            yaw = result.getBestTarget().getYaw();
        }
        return yaw;
    }

    public double getPitch() {
        var result = getLatestCameraResult();
        double pitch = 0.0;
        if (result.hasTargets()) {
            // Calculate angular turn power
            // Remove -1.0 because it was inverting results.
            pitch = result.getBestTarget().getPitch();
        }

        return pitch;
    }

}

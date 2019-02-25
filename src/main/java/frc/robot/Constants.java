/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {
    /**
     * set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int kTimeoutMs = 30;

    /**
     * Gains used in Motion Magic, to be adjusted accordingly Gains(kp, ki, kd, kf,
     * izone, peak output);
     */
    public static final double kP = 0.2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.2;
    public static final double izone = 0;
    public static final double peak_output = 1.0;

    // Field zero-degree measure
    public static final double field_zero = 0.0;

    // Sensitivity
    public static final double throttle_sensitivity = 0.5;
    public static final double turn_sensitivity = 0.5;

    // Motion
    public static final double max_acceleration = 0.5;

    // Vision
    public static final int camera_center = 80;
}

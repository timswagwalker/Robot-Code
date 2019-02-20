/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * Add your docs here.
 */
public class Instrumentation {
    public static AHRS navx;
    public static Encoder e_left, e_right, e_elevator, e_arm_angle;

    public static void init() {
        // Define encoders and NavX-MXP
        navx = new AHRS(Port.kMXP);

        e_right = new Encoder(RobotMap.e_right_port_1, RobotMap.e_right_port_2, false, Encoder.EncodingType.k4X);
        e_left = new Encoder(RobotMap.e_left_port_1, RobotMap.e_left_port_2, true, Encoder.EncodingType.k4X);
        e_elevator = new Encoder(RobotMap.e_elevator_port_1, RobotMap.e_elevator_port_2, false, Encoder.EncodingType.k4X);
        e_arm_angle = new Encoder(RobotMap.e_arm_port_1, RobotMap.e_arm_port_2, false, Encoder.EncodingType.k4X);

        navx.reset();
        e_right.reset();
        e_left.reset();
        e_elevator.reset();
        e_arm_angle.reset();
    }
}

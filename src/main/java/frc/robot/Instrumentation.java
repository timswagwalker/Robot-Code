/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * Add your docs here.
 */
public class Instrumentation {
    public static AHRS navx;
    public static Encoder e_left, e_right, e_elevator, e_arm_angle;
    public static DigitalInput l_extend_front, l_extend_rear, l_retract_front, l_retract_rear;

    public static void init() {
        // Define encoders and NavX-MXP
        navx = new AHRS(Port.kMXP);

        e_right = new Encoder(RobotMap.e_right_port_1, RobotMap.e_right_port_2, true, Encoder.EncodingType.k4X);
        e_left = new Encoder(RobotMap.e_left_port_1, RobotMap.e_left_port_2, false, Encoder.EncodingType.k4X);
        // e_elevator = new Encoder(RobotMap.e_elevator_port_1, RobotMap.e_elevator_port_2, false, Encoder.EncodingType.k4X);
        // e_arm_angle = new Encoder(RobotMap.e_arm_port_1, RobotMap.e_arm_port_2, false, Encoder.EncodingType.k4X);

        // l_extend_front = new DigitalInput(RobotMap.l_extend_front_port);
        // l_extend_rear = new DigitalInput(RobotMap.l_extend_rear_port);
        // l_retract_front = new DigitalInput(RobotMap.l_retract_front_port);
        // l_retract_rear = new DigitalInput(RobotMap.l_retract_rear_port);

        navx.reset();
        e_right.reset();
        e_left.reset();
        // e_elevator.reset();
        // e_arm_angle.reset();

        e_right.setDistancePerPulse(0.0047);            // inches
        e_left.setDistancePerPulse(0.0047);             // inches
        // e_arm_angle.setDistancePerPulse(11.1111);       // degrees
    }
}

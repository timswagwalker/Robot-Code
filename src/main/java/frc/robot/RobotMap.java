/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  // Drivetrain controller ports
  public static int m_right_front_port = 3;
  public static int m_right_rear_port = 4;
  public static int m_left_rear_port = 1;
  public static int m_left_front_port = 2;

  // Elevator controller ports
  public static int m_elevator_port = 5;

  // Grabber controller ports
  public static int m_arm_port = 6;
  public static int m_intake_1_port = 7;
  public static int m_intake_2_port = 8;

  // Climber controller ports
  public static int m_climber_front_port = 9;
  public static int m_climber_rear_port = 10;
  public static int m_climber_rear_drive_port = 11;

  // Servo ports
  public static int s_front_camera_port = 0;

  // Encoder ports
  public static int e_right_port_1 = 0;
  public static int e_right_port_2 = 1;
  public static int e_left_port_1 = 2;
  public static int e_left_port_2 = 3;
  public static int e_elevator_port_1 = 4;
  public static int e_elevator_port_2 = 5;
  public static int e_arm_port_1 = 6;
  public static int e_arm_port_2 = 7;

  // Solenoid channels
  public static int p_grabber_channel_fwd = 0;
  public static int p_grabber_channel_rev = 4;

  // Limit Switch channels
  public static int l_extend_front_port = 1;
  public static int l_extend_rear_port = 2;
  public static int l_retract_front_port = 3;
  public static int l_retract_rear_port = 4;

  // Joystick button IDs
  public static int brake_button = 1;
  public static int manual_button = 2;
  public static int grab_button = 2;
  public static int climb_hab_button = 7;
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.commands.Aim;

/**
 * Add your docs here.
 */
public class Grabber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static WPI_VictorSPX m_arm, m_intake_1, m_intake_2;
  static DoubleSolenoid s_grabber;

  public Boolean is_open = false;

  public Grabber() {
    // Define, reset, and don't invert the motor controllers
    m_arm = new WPI_VictorSPX(RobotMap.m_arm_port);
    m_intake_1 = new WPI_VictorSPX(RobotMap.m_intake_1_port);
    m_intake_2 = new WPI_VictorSPX(RobotMap.m_intake_2_port);

    m_arm.configFactoryDefault();
    m_intake_1.configFactoryDefault();
    m_intake_2.configFactoryDefault();

    m_arm.setInverted(false);
    m_intake_1.setInverted(false);
    m_intake_2.setInverted(false);

    // Define the double solenoid
    s_grabber = new DoubleSolenoid(RobotMap.s_grabber_channel_fwd, RobotMap.s_grabber_channel_rev);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Aim());
  }

  // Open the grabber
  public void openGrabber() {
    s_grabber.set(Value.kForward);
  }

  // Close the grabber
  public void closeGrabber() {
    s_grabber.set(Value.kReverse);
  }

  // Run the intake motors
  public void runIntake(double direction) {
    m_intake_1.set(direction);
    m_intake_2.set(direction);
  }

  // Turn the grabber arm to the desired angle
  public void setAngle(double speed) {
    // Insert code for turning the arm based on joystick input and the encoder in Instrumentation
    m_arm.set(ControlMode.PercentOutput, speed);
  }
}

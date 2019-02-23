/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

// extra libraries
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.Instrumentation;

// commands
import frc.robot.commands.Drive;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static WPI_TalonSRX m_left_front, m_left_follower, m_right_front, m_right_follower;
  static double max_acceleration;

  public Drivetrain() {
    // Define motors
    m_left_front = new WPI_TalonSRX(RobotMap.m_left_front_port);
    m_left_follower = new WPI_TalonSRX(RobotMap.m_left_rear_port);
    m_right_front = new WPI_TalonSRX(RobotMap.m_right_front_port);
    m_right_follower = new WPI_TalonSRX(RobotMap.m_right_rear_port);

    // Reset to factory defaults to prevent unexpected behavior
    m_left_front.configFactoryDefault();
    m_left_follower.configFactoryDefault();
    m_right_front.configFactoryDefault();
    m_right_follower.configFactoryDefault();

    /*
     * // Set encoders for the main Talons
     * m_left_front.configSelectedFeedbackSensor(RemoteFeedbackDevice.
     * SoftwareEmulatedSensor);
     * m_right_front.configSelectedFeedbackSensor(RemoteFeedbackDevice.
     * SoftwareEmulatedSensor);
     *
     * // Phase sensors for the main Talons m_left_front.setSensorPhase(true);
     * m_right_front.setSensorPhase(true);
     */

    // Invert right motors
    m_left_front.setInverted(false);
    m_left_follower.setInverted(false);
    m_right_front.setInverted(true);
    m_right_follower.setInverted(true);

    // Set rear Talons to follow front Talons
    m_left_follower.follow(m_left_front);
    m_right_follower.follow(m_right_front);

    // Set the cruise velocity and acceleration
    max_acceleration = 1; // joystick_units/sec^2

    /* setupMotionMagic(); */
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new Drive());
  }

  // Motion Magic specific setup routines
  public void setupMotionMagic() {
    // Set frame periods so Motion Magic works at the periodic rate
    m_left_front.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    m_left_front.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    m_right_front.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    m_right_front.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    // Set peak and nominal outputs
    m_left_front.configNominalOutputForward(0, Constants.kTimeoutMs);
    m_left_front.configNominalOutputReverse(0, Constants.kTimeoutMs);
    m_left_front.configPeakOutputForward(1, Constants.kTimeoutMs);
    m_left_front.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    m_right_front.configNominalOutputForward(0, Constants.kTimeoutMs);
    m_right_front.configNominalOutputReverse(0, Constants.kTimeoutMs);
    m_right_front.configPeakOutputForward(1, Constants.kTimeoutMs);
    m_right_front.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    // Set Motion Magic gains
    m_left_front.selectProfileSlot(0, 0);
    m_left_front.config_kF(0, Constants.kF, Constants.kTimeoutMs);
    m_left_front.config_kP(0, Constants.kP, Constants.kTimeoutMs);
    m_left_front.config_kI(0, Constants.kI, Constants.kTimeoutMs);
    m_left_front.config_kD(0, Constants.kD, Constants.kTimeoutMs);

    m_right_front.selectProfileSlot(0, 0);
    m_right_front.config_kF(0, Constants.kF, Constants.kTimeoutMs);
    m_right_front.config_kP(0, Constants.kP, Constants.kTimeoutMs);
    m_right_front.config_kI(0, Constants.kI, Constants.kTimeoutMs);
    m_right_front.config_kD(0, Constants.kD, Constants.kTimeoutMs);

    // Set acceleration and cruise velocity
    m_left_front.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    m_left_front.configMotionAcceleration(6000, Constants.kTimeoutMs);

    m_right_front.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    m_right_front.configMotionAcceleration(6000, Constants.kTimeoutMs);

    // Zero Talon sensors
    m_left_front.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    m_right_front.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
  }

  // Get motion profile speeds
  public double getMotion(double target, Boolean is_left) {
    // Make motion profiling to work on target velocity set by the joystick, predefined acceleration, and should ramp up velocity to the target.
    double target_vel = target;
    double current_vel = 0.0;
    if (is_left) {
      current_vel = Instrumentation.e_left.getRate(); // inches/sec
    } else {
      current_vel = Instrumentation.e_right.getRate(); // inches/sec
    }

    double output = 0.0;
    if (Math.abs(target_vel - current_vel) <= 0.05) {
      output = target_vel;
    } else {
      output = current_vel + (0.2 * max_acceleration);
    }

    return output;
  }

  // <CRUISE CONTROL mode> Drive with Motion Profile assist
  public void driveAssist(double left, double right) {
    // Get target velocities for each side of the drivebase
    double left_target_vel = left;
    double right_target_vel = right;

    // Assign the target velocities to each side of the drivebase with Motion Magic
    // m_left_front.set(ControlMode.MotionMagic, left_target_pos);
    // m_right_front.set(ControlMode.MotionMagic, right_target_pos);
    m_left_front.set(ControlMode.PercentOutput, getMotion(left_target_vel, true));
    m_right_front.set(ControlMode.PercentOutput, getMotion(right_target_vel, false));

    /* 10 Ms timeout, allow CAN Frames to process */
    try {
      TimeUnit.MILLISECONDS.sleep(10);
    } catch (Exception e) {
      /* Do Nothing */ }
  }

  // <HYPER STEER mode> Drive manually without Motion Profiling
  public void driveManual(double left, double right) {
    m_left_front.set(ControlMode.PercentOutput, left);
    m_right_front.set(ControlMode.PercentOutput, right);
  }

  // Stop all movement
  public void brake() {
    m_left_front.set(0);
    m_left_follower.set(0);
    m_right_front.set(0);
    m_right_follower.set(0);
  }

}

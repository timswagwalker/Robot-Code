/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_VictorSPX m_lifter_front, m_lifter_rear, m_rear_drive;

  public Climber() {
    // m_lifter_front = new WPI_VictorSPX(RobotMap.m_lifter_front_port);
    // m_lifter_rear = new WPI_VictorSPX(RobotMap.m_lifter_rear_port);
    // m_rear_drive = new WPI_VictorSPX(RobotMap.m_lifter_rear_drive_port);

    // m_lifter_front.setInverted(false);
    // m_lifter_rear.setInverted(false);
    // m_rear_drive.setInverted(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setFrontLifter(double speed) {
    m_lifter_front.set(ControlMode.PercentOutput, speed);
  }

  public void setRearLifter(double speed) {
    m_lifter_rear.set(ControlMode.PercentOutput, speed);
  }

  public void setRearDrive(double speed) {
    m_rear_drive.set(ControlMode.PercentOutput, speed);
  }
}

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
import frc.robot.RobotMap;
import frc.robot.commands.Lift;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static WPI_VictorSPX m_elevator;

  public Elevator() {
    // Define, reset, and don't invert the motor controllers
    m_elevator = new WPI_VictorSPX(RobotMap.m_elevator_port);
    m_elevator.configFactoryDefault();
    m_elevator.setInverted(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Lift());
  }

  // Move the elevator
  public void move(double speed) {
    m_elevator.set(ControlMode.PercentOutput, speed);
  }

  // Stop all movement
  public void brake() {
    m_elevator.set(0);
  }
}

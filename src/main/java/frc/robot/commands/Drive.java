/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;

public class Drive extends Command {
  public Drive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Set left and right speed values
    double leftSpeed = Robot.oi.getLeftSpeed();
    double rightSpeed = Robot.oi.getRightSpeed();

    if (Robot.oi.getDriveBrake()) {
      // Stop all drivetrain commands
      Robot.drivetrain.driveManual(0, 0);
    } else {
      if (Robot.oi.getControlMode()) {
        // <HYPER SWERVE mode> Drive in manual mode
        Robot.drivetrain.driveManual(leftSpeed, rightSpeed);
      } else {
        // <CRUISE CONTROL mode> Drive in Motion Profile assisted mode
        Robot.drivetrain.driveAssist(leftSpeed, rightSpeed);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // Add a button toggle if the joystick is to be switched from drive mode to
    // another mode
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

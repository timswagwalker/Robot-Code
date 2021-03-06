/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Instrumentation;
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
    Robot.drivetrain.driveManual(0, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Set left and right speed values
    double throttle = Robot.oi.getThrottle();
    double turn = Robot.oi.getTurn();

    double error, kp_correction, ki_correction = 0.0;

    error = Instrumentation.navx.getRate() - turn;
    kp_correction = 0.1 * error;
    ki_correction += 0.01 * error;

    turn += kp_correction + ki_correction;

    double leftSpeed = throttle + turn;
    double rightSpeed = throttle - turn;

    if (Robot.oi.getDriveBrake()) {
      // Stop all drivetrain commands
      Robot.drivetrain.driveManual(0, 0);
    } else {
      if (Robot.oi.getControlMode()) {
        // <HYPER STEER mode> Drive in manual mode
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
    Robot.drivetrain.driveManual(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

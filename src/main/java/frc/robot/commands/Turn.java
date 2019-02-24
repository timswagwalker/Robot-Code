/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Instrumentation;
import frc.robot.Robot;

public class Turn extends Command {
  private double left_speed;
  private double right_speed;
  private double current_angle;
  private double target_angle;

  public Turn(int angle, Boolean is_robot_centric) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    if (angle > 180) {
      angle = angle - 360;
    }

    if (is_robot_centric) {
      // Robot-centric target
      target_angle = Instrumentation.navx.getAngle() + angle;
    } else {
      // Field-centric target
      target_angle = Constants.field_zero + angle;
    }

    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.oi.getControlMode()) {
      // Robot-centric turn control
      current_angle = Instrumentation.navx.getAngle();

      left_speed = (target_angle - current_angle) / 180;
      right_speed = (current_angle - target_angle) / 180;

      Robot.drivetrain.driveAssist(left_speed * 10, right_speed * 10);
    } else {
      // Field-centric turn control
      current_angle = Instrumentation.navx.getAngle() % 360;

      left_speed = (target_angle - current_angle) / 180;
      right_speed = (current_angle - target_angle) / 180;

      Robot.drivetrain.driveAssist(left_speed * 10, right_speed * 10);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(target_angle - current_angle) <= 0.1;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

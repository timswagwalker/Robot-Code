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

public class Aim extends Command {
  private double target;
  private double current_angle;

  public Aim() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.grabber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    target = (Robot.oi.getSlider() + 1) * 75;
    current_angle = Instrumentation.e_arm_angle.getDistance();

    double speed;
    speed = (target - current_angle) / target;

    Robot.grabber.setAngle(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(target - current_angle) < 0.1;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.grabber.setAngle(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

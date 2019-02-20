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

public class Retract extends Command {
  private Boolean front;

  public Retract(Boolean is_front) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.front = is_front;
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (front) {
      Robot.climber.setFrontLifter(-1.0);
    } else {
      Robot.climber.setRearLifter(-1.0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (front) {
      return Instrumentation.l_retract_front.get();
    } else {
      return Instrumentation.l_retract_rear.get();
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (front) {
      Robot.climber.setFrontLifter(0.0);
    } else {
      Robot.climber.setRearLifter(0.0);
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

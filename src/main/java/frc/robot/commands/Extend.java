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

public class Extend extends Command {
  public Extend() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double current_angle = Instrumentation.navx.getPitch();

    double front_speed = -1.0 * ((current_angle / 30) + 0.75);
    double rear_speed = +1.0 * ((current_angle / 30) + 0.75);

    if (Instrumentation.l_extend_front.get()) {
      front_speed = 0;
    }
    if (Instrumentation.l_extend_rear.get()) {
      rear_speed = 0;
    }

    Robot.climber.setFrontLifter(front_speed);
    Robot.climber.setRearLifter(rear_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Instrumentation.l_extend_front.get() && Instrumentation.l_extend_rear.get() && Math.abs(Instrumentation.navx.getPitch()) < 1.0;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climber.setFrontLifter(0.0);
    Robot.climber.setRearLifter(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

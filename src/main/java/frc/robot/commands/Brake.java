/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Brake extends InstantCommand {
  private Boolean drive;

  /**
   * Add your docs here.
   */
  public Brake(Boolean drive) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.drive = drive;
    if (drive) {
      requires(Robot.drivetrain);
    } else {
      requires(Robot.elevator);
    }
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (drive) {
      Robot.drivetrain.driveManual(0, 0);
    } else {
      Robot.elevator.move(0 );
    }
  }

}

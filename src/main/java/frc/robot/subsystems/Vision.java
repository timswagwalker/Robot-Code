/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Instrumentation;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static Servo s_camera;

  public double servo_angle, target_x, target_angle;

  double tape1 = Constants.camera_center;
  double tape2 = Constants.camera_center;
  boolean tape1_is_visible = true;
  boolean tape2_is_visible = true;

  public Vision() {
    s_camera = new Servo(RobotMap.s_front_camera_port);
    servo_angle = s_camera.getAngle();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public long last_seen = 0;

  public void getTarget() {
    double[] default_array = { -1, -1 };
    double temp;

    temp = SmartDashboard.getNumberArray("tape1", default_array)[0];
    if (temp != -1) {
      tape1 = temp;
    } else {
      tape1_is_visible = false;
    }

    temp = SmartDashboard.getNumberArray("tape2", default_array)[0];
    if (temp != -1) {
      tape2 = temp;
    } else {
      tape2_is_visible = false;
    }

    if (tape1_is_visible || tape2_is_visible) {
      last_seen = System.currentTimeMillis();
    }

    target_x = (tape1 + tape2) / 2;
  }

  public void trackServo() {
    getTarget();
    double error = Constants.camera_center - target_x;
    if (tape1_is_visible || tape2_is_visible) {
      servo_angle += Math.abs(error) > 3 ? error * 0.03 : 0.0;
    }

    s_camera.setAngle(servo_angle);
  }

  public double getTargetAngle() {
    trackServo();
    target_angle = (s_camera.getAngle() - 90) * -1 + (target_x - Constants.camera_center) * 0.3152;

    double sensitivity = 2.5;
    double overshoot = sensitivity * (Instrumentation.getHeading() + target_angle);

    if (overshoot >= 80) {
      overshoot = 80;
    } else if (overshoot <= -80) {
      overshoot = -80;
    }

    return overshoot;
  }
}

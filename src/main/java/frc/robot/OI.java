/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.RobotMap;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  Joystick driveStick = new Joystick(1);
  Joystick liftStick = new Joystick(2);

  Button driveBrake = new JoystickButton(driveStick, RobotMap.brake_button);
  Button liftBrake = new JoystickButton(liftStick, RobotMap.brake_button);
  Button manual = new JoystickButton(driveStick, RobotMap.manual_button);
  Button grab = new JoystickButton(liftStick, RobotMap.grab_button);

  public double deadband(double input) {
    if (input > +0.05)
      return input - 0.05;

    if (input < -0.05)
      return input + 0.05;

    return 0.0;
  }

  // Drive Methods

  public double getThrottle() {
    double throttle = -1.0 * driveStick.getY() * Constants.throttle_sensitivity;
    throttle = deadband(throttle);
    return throttle;
  }

  public double getTurn() {
    double turn = +1.0 * driveStick.getZ() * Constants.turn_sensitivity;
    turn = deadband(turn);
    return turn;
  }

  public Boolean getControlMode() {
    // if more modes need to be added, add more cases/switches
    if (manual.get()) {
      return true; // Manual drive control / Robot-centric turn control
    } else {
      return false; // Motion Magic assisted drive control / Field-centric turn control
    }
  }

  public Boolean getDriveBrake() {
    if (driveBrake.get()) {
      return true;
    } else {
      return false;
    }
  }

  public int getTurnAngle() {
    return driveStick.getPOV();
  }

  // Elevator Methods

  public double getElevatorSpeed() {
    double speed = -1.0 * liftStick.getY();
    speed = deadband(speed);

    return speed;
  }

  public Boolean getLiftBrake() {
    if (liftBrake.get()) {
      return true;
    } else {
      return false;
    }
  }

  // Grabber methods

  public int getIntakeDirection() {
    int angle = liftStick.getPOV();

    if (angle == 0) {
      return 1;
    } else if (angle == 180) {
      return -1;
    } else {
      return 0;
    }
  }

  public Boolean getGrab() {
    if (grab.get()) {
      return true;
    } else {
      return false;
    }
  }

  public double getSlider() {
    return liftStick.getThrottle();
  }
}

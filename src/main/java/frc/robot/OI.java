package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class OI {
  private static EventLoop eventLoop = new EventLoop();
  public static CommandXboxController primaryController = new CommandXboxController(0);
  public static Joystick leftButtonBoard = new Joystick(1);
  public static Joystick rightButtonBoard = new Joystick(2);

  public static void update() {
    eventLoop.poll();
  }

  public static double deadband(double input, double deadband) {
    if (Math.abs(input) > deadband) {
      if (input > 0.0) {
        return (input - deadband) / (1.0 - deadband);
      }

      return (input + deadband) / (1.0 - deadband);
    }

    return 0.0;
  }

  public static final class Drive {
    // X is forward from drivers' perspective
    public static double getX() {
      return -primaryController.getLeftY();
    }

    // Y is left from drivers' perspective
    public static double getY() {
      return -primaryController.getLeftX();
    }

    public static double getOmega() {
      return -primaryController.getRightX();
    }

    public static Trigger stopWithX() {
      return primaryController.x(eventLoop);
    }

    public static Trigger rezeroGyro() {
      return primaryController.back(eventLoop);
    }
  }
}

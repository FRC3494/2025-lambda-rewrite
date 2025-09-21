package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class OI {
  private static EventLoop eventLoop = new EventLoop();
  private static CommandXboxController primaryController = new CommandXboxController(0);
  private static Joystick leftButtonBoard = new Joystick(1);
  private static Joystick rightButtonBoard = new Joystick(2);

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

  // TODO: toggle defense sensor?
  // TODO: auto align

  // ==================== Stow ====================
  public static Trigger stow() {
    return rightButtonBoard.button(6, eventLoop).castTo(Trigger::new);
  }

  // ==================== Ground Intake ====================
  public static Trigger groundIntakeForTransfer() {
    return primaryController.rightTrigger(0.05, eventLoop);
  }

  public static Trigger groundIntakeForL1() {
    return primaryController.a(eventLoop);
  }

  public static Trigger groundIntakeL1Outtake() {
    return rightButtonBoard.button(10, eventLoop).castTo(Trigger::new);
  }

  public static Trigger groundIntakeManualOuttake() {
    return rightButtonBoard.button(8, eventLoop).castTo(Trigger::new);
  }

  // ==================== Coral Intake w/ Arm ====================
  public static Trigger feeder() {
    return leftButtonBoard.button(6, eventLoop).castTo(Trigger::new);
  }

  // ==================== Coral Outtake w/ Arm ====================
  public static Trigger armL1Coral() {
    return leftButtonBoard.button(10, eventLoop).castTo(Trigger::new);
  }

  public static Trigger L2Coral() {
    return leftButtonBoard.button(5, eventLoop).castTo(Trigger::new);
  }

  public static Trigger L3Coral() {
    return leftButtonBoard.button(2, eventLoop).castTo(Trigger::new);
  }

  // ==================== Algae Intake ====================
  public static Trigger L2Algae() {
    return leftButtonBoard.button(4, eventLoop).castTo(Trigger::new);
  }

  public static Trigger L3Algae() {
    return leftButtonBoard.button(1, eventLoop).castTo(Trigger::new);
  }

  // ==================== Algae Outtake ====================
  public static Trigger processor() {
    return leftButtonBoard.button(8, eventLoop).castTo(Trigger::new);
  }

  public static Trigger preBarge() {
    return leftButtonBoard.button(9, eventLoop).castTo(Trigger::new);
  }

  public static Trigger barge() {
    return leftButtonBoard.button(7, eventLoop).castTo(Trigger::new);
  }

  // ==================== Climb ====================
  public static Trigger preClimb() {
    return rightButtonBoard.button(1, eventLoop).castTo(Trigger::new);
  }

  public static Trigger climbStage1() {
    return rightButtonBoard.button(2, eventLoop).castTo(Trigger::new);
  }

  public static Trigger climbStage2() {
    return rightButtonBoard.button(3, eventLoop).castTo(Trigger::new);
  }

  public static Trigger climbStage3() {
    return rightButtonBoard.button(4, eventLoop).castTo(Trigger::new);
  }
}

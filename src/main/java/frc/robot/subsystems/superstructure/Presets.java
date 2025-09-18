package frc.robot.subsystems.superstructure;

public final class Presets {
  public static class Default {
    public static final double elevatorHeight = 0.0;

    public static final double armAngle = 0.75;
    public static final double intakeSpeed = 0.0;

    public static final double groundIntakeAngle = 0.31;
    public static final double groundIntakeFrontSpeed = 0.0;
    public static final double groundIntakeBackSpeed = 0.0;
  }

  public static class Stow {
    public static final double elevatorHeight = 0.0;

    public static final double armAngle = 0.75;
  }

  // ==================== Ground Intake ====================
  public static class GroundIntakeForTransfer {
    public static final double elevatorHeight = 0.0;

    public static final double armAngle = 0.959;
  }

  public static class GroundIntakeForL1 {
    public static final double elevatorHeight = 0.0;

    public static final double armAngle = 0.75;
  }

  public static class GroundIntakeL1 {
    public static final double elevatorHeight = 0.0;

    public static final double armAngle = 0.75;
  }

  public static class GroundIntakeL1Jerk {
    public static final double elevatorHeight = 0.0;

    public static final double armAngle = 0.75;
  }

  // ==================== Coral Outtake w/ Arm ====================
  public static class CoralOuttake {
    // TODO: intake speed
  }

  public static class L2Coral {
    public static final double armAngle = 0.610;
  }

  public static class L3Coral {
    public static final double elevatorHeight = 44.5;

    public static final double armAngle = 0.63;

    public static final double groundIntakeAngle = 0.31;
  }

  // ==================== Algae Intake ====================
  public static class L2Algae {
    public static final double armAngle = 0.62;
  }

  public static class L3Algae {
    public static final double elevatorHeight = 28.75;

    public static final double armAngle = 0.632;

    public static final double groundIntakeAngle = 0.31;
  }

  // ==================== Algae Outtake ====================
  public static class Processor {}

  public static class PreBarge {}

  public static class Barge {}
}

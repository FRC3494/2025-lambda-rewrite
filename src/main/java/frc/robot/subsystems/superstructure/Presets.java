package frc.robot.subsystems.superstructure;

public final class Presets {
  public static class Default {
    public static final double elevatorHeight = 0.0;

    public static final double armAngle = 0.72;
    public static final double intakeSpeed = 0.0; // Positive is algae intake / coral outtake

    public static final double groundIntakeAngle = 0.31;
    public static final double groundIntakeFrontSpeed = 0.0;
    public static final double groundIntakeBackSpeed = 0.0;
  }

  public static final class IntakeSpeeds {
    // TODO: tune
    public static final double coralPassiveIntake = -0.3;
    public static final double coralActiveIntake = -1.0;
    public static final double coralOuttake = 1.0;

    public static final double algaePassiveIntake = 0.5;
    public static final double algaeActiveIntake = 1.0;
    public static final double algaeOuttake = -1.0;
  }

  public static class Stow {
    public static final double elevatorHeight = 0.0;

    public static final double armAngle = 0.72;
    public static final double intakeSpeed = IntakeSpeeds.coralPassiveIntake;

    public static final double groundIntakeAngle = 0.31;
    public static final double groundIntakeFrontSpeed = 0.0;
    public static final double groundIntakeBackSpeed = 0.0;
  }

  // ==================== Ground Intake ====================
  // TODO: non-defense mode ground intake hover position is 0.05

  public static class GroundIntakeForTransfer {
    public static final double elevatorHeight = 0.0;

    public static final double armAngle = 0.959;
    public static final double intakeSpeed = IntakeSpeeds.coralActiveIntake;

    public static final double groundIntakeAngle = 0.0312;
    public static final double groundIntakeFrontSpeed = -0.85;
    public static final double groundIntakeBackSpeed = 0.85;
  }

  public static class DoneWithGroundIntakeForTransfer {
    public static final double elevatorHeight = GroundIntakeForTransfer.elevatorHeight;

    public static final double armAngle = GroundIntakeForTransfer.armAngle;
    public static final double intakeSpeed = IntakeSpeeds.coralPassiveIntake;

    public static final double groundIntakeAngle = 0.05;
    public static final double groundIntakeFrontSpeed = 0.0;
    public static final double groundIntakeBackSpeed = 0.0;
  }

  public static class GroundIntakeForL1 {
    public static final double elevatorHeight = Stow.elevatorHeight;

    public static final double armAngle = Stow.armAngle;
    public static final double intakeSpeed = 0.0;

    public static final double groundIntakeAngle = 0.0312;
    public static final double groundIntakeFrontSpeed = -0.85;
    public static final double groundIntakeBackSpeed = -0.6;
  }

  public static class GroundIntakeL1 {
    public static final double elevatorHeight = Stow.elevatorHeight;

    public static final double armAngle = Stow.armAngle;

    public static final double groundIntakeAngle = 0.29;
    public static final double groundIntakeFrontSpeed = 0.0;
    public static final double groundIntakeBackSpeed = 0.0;
  }

  public static class GroundIntakeL1PreJerk {
    public static final double elevatorHeight = Stow.elevatorHeight;

    public static final double armAngle = Stow.armAngle;

    public static final double groundIntakeAngle = 0.36;
    public static final double groundIntakeFrontSpeed = 0.25;
    public static final double groundIntakeBackSpeed = -0.25;
  }

  public static class GroundIntakeL1Jerk {
    public static final double elevatorHeight = Stow.elevatorHeight;

    public static final double armAngle = Stow.armAngle;

    public static final double groundIntakeAngle = 0.27;
  }

  // ==================== Coral Intake w/ Arm ====================
  public static class Feeder {
    public static final double elevatorHeight = 10.0;

    public static final double armAngle = 0.845;
    public static final double intakeSpeed = IntakeSpeeds.coralActiveIntake;

    public static final double groundIntakeAngle = 0.31;
    public static final double groundIntakeFrontSpeed = 0.0;
    public static final double groundIntakeBackSpeed = 0.0;
  }

  // ==================== Coral Outtake w/ Arm ====================
  // TODO: old old (original) coral L1 arm angle is 0.875

  public static class ArmL1Coral {
    public static final double elevatorHeight = 8.38;

    public static final double armAngle = 0.613;

    public static final double groundIntakeAngle = Stow.groundIntakeAngle;
  }

  public static class L2Coral {
    public static final double elevatorHeight = 20.0;

    public static final double armAngle = 0.610;

    public static final double groundIntakeAngle = Stow.groundIntakeAngle;
  }

  public static class L3Coral {
    public static final double elevatorHeight = 44.5;

    public static final double armAngle = 0.63;

    public static final double groundIntakeAngle = Stow.groundIntakeAngle;
  }

  // ==================== Algae Intake ====================
  // TODO: lolipop at some point?

  public static class L2Algae {
    public static final double elevatorHeight = 0.0;

    public static final double armAngle = 0.62;
    public static final double intakeSpeed = IntakeSpeeds.algaeActiveIntake;

    public static final double groundIntakeAngle = Stow.groundIntakeAngle;
  }

  public static class L3Algae {
    public static final double elevatorHeight = 28.75;

    public static final double armAngle = 0.632;
    public static final double intakeSpeed = IntakeSpeeds.algaeActiveIntake;

    public static final double groundIntakeAngle = Stow.groundIntakeAngle;
  }

  // ==================== Algae Outtake ====================
  public static class Processor {
    public static final double elevatorHeight = 0.0;

    public static final double armAngle = 0.54;
    public static final double intakeSpeed = IntakeSpeeds.algaePassiveIntake;

    public static final double groundIntakeAngle = Stow.groundIntakeAngle;
  }

  public static class ProcessorOuttake {
    public static final double intakeSpeed = IntakeSpeeds.algaeOuttake;
  }

  public static class PreBarge {
    public static final double elevatorHeight = 44.5;

    public static final double armAngle = 0.85;
    public static final double intakeSpeed = IntakeSpeeds.algaePassiveIntake;

    public static final double groundIntakeAngle = Stow.groundIntakeAngle;
  }

  public static class Barge {
    public static final double elevatorHeight = 44.5;

    public static final double armAngle = 0.65;
    public static final double intakeSpeed = IntakeSpeeds.algaeOuttake;

    public static final double groundIntakeAngle = Stow.groundIntakeAngle;
  }

  // ==================== Pre-Climb ====================
  public static class PreClimb {
    public static final double elevatorHeight = 20.0;

    public static final double armAngle = 0.56;
    public static final double intakeSpeed = 0.0;

    public static final double groundIntakeAngle = 0.05;
    public static final double groundIntakeFrontSpeed = 0.0;
    public static final double groundIntakeBackSpeed = 0.0;
  }
}

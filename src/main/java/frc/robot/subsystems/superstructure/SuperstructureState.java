package frc.robot.subsystems.superstructure;

import lombok.Getter;

@Getter
public enum SuperstructureState {
  IDLE(SuperstructureStateData.builder().build()),

  STOW(
      SuperstructureStateData.builder()
          .elevatorHeight(0.0) // Positive is upwards
          .armAngle(0.72) // TODO: Positive is toward (processor side or G-intake side)
          .intakeSpeed(IntakeSpeeds.coralPassiveIntake) // Positive is algae intake / coral outtake
          .groundIntakeAngle(0.31) // Positive is upwards
          .groundIntakeFrontSpeed(0.0) // Positive is outtake
          .groundIntakeBackSpeed(0.0) // Positive is intake
          .build()),

  // ==================== Ground Intake ====================
  // TODO: non-defense mode ground intake hover position is 0.05
  GROUND_INTAKE_ARM_IN_BETWEEN(
      SuperstructureStateData.builder()
          // TODO
          .elevatorHeight(null)
          .armAngle(null)
          .intakeSpeed(IntakeSpeeds.coralPassiveIntake)
          .groundIntakeAngle(null)
          .groundIntakeFrontSpeed(0.0)
          .groundIntakeBackSpeed(0.0)
          .build()),
  PRE_GROUND_INTAKE_FOR_TRANSFER(
      SuperstructureStateData.builder()
          .elevatorHeight(0.0)
          .armAngle(0.959)
          .intakeSpeed(IntakeSpeeds.coralActiveIntake)
          .groundIntakeAngle(0.0312)
          .groundIntakeFrontSpeed(0.0)
          .groundIntakeBackSpeed(0.0)
          .build()),
  GROUND_INTAKE_FOR_TRANSFER(
      PRE_GROUND_INTAKE_FOR_TRANSFER.getValue().toBuilder()
          .groundIntakeFrontSpeed(-0.85)
          .groundIntakeBackSpeed(0.85)
          .build()),
  DONE_WITH_GROUND_INTAKE_FOR_TRANSFER(
      GROUND_INTAKE_FOR_TRANSFER.getValue().toBuilder()
          .intakeSpeed(IntakeSpeeds.coralPassiveIntake)
          .groundIntakeAngle(0.05)
          .groundIntakeFrontSpeed(0.0)
          .groundIntakeBackSpeed(0.0)
          .build()),

  GROUND_INTAKE_FOR_L1(
      STOW.getValue().toBuilder()
          .intakeSpeed(0.0)
          .groundIntakeAngle(0.0312)
          .groundIntakeFrontSpeed(-0.85)
          .groundIntakeBackSpeed(-0.6)
          .build()),
  GROUND_INTAKE_L1(
      GROUND_INTAKE_FOR_L1.getValue().toBuilder()
          .groundIntakeAngle(0.29)
          .groundIntakeFrontSpeed(0.0)
          .groundIntakeBackSpeed(0.0)
          .build()),
  GROUND_INTAKE_L1_PRE_JERK(
      GROUND_INTAKE_FOR_L1.getValue().toBuilder()
          .groundIntakeAngle(0.36)
          .groundIntakeFrontSpeed(0.25)
          .groundIntakeBackSpeed(-0.25)
          .build()),
  GROUND_INTAKE_L1_JERK(
      GROUND_INTAKE_FOR_L1.getValue().toBuilder()
          .groundIntakeAngle(0.27)
          .groundIntakeFrontSpeed(0.25)
          .groundIntakeBackSpeed(-0.25)
          .build()),

  // ==================== Coral Intake w/ Arm ====================
  PRE_FEEDER(
      SuperstructureStateData.builder()
          .elevatorHeight(10.0)
          .armAngle(0.72)
          .groundIntakeAngle(0.31)
          .groundIntakeFrontSpeed(0.0)
          .groundIntakeBackSpeed(0.0)
          .build()),
  FEEDER(
      PRE_FEEDER.getValue().toBuilder()
          .armAngle(0.845)
          .intakeSpeed(IntakeSpeeds.coralActiveIntake)
          // Comment for autoformat
          .build()),

  // ==================== Coral Outtake w/ Arm ====================
  ARM_L1_CORAL(
      SuperstructureStateData.builder()
          .elevatorHeight(8.38)
          .armAngle(0.613)
          // Comment for autoformat
          .build()),
  ARM_L1_CORAL_OUTTAKE(
      ARM_L1_CORAL.getValue().toBuilder()
          .intakeSpeed(IntakeSpeeds.coralOuttake)
          // Comment for autoformat
          .build()),
  L2_CORAL(
      SuperstructureStateData.builder()
          .elevatorHeight(20.0)
          .armAngle(0.610)
          // Comment for autoformat
          .build()),
  L2_CORAL_OUTTAKE(
      L2_CORAL.getValue().toBuilder()
          .intakeSpeed(IntakeSpeeds.coralOuttake)
          // Comment for autoformat
          .build()),
  L3_CORAL(
      SuperstructureStateData.builder()
          .elevatorHeight(44.5)
          .armAngle(0.63)
          // Comment for autoformat
          .build()),
  L3_CORAL_OUTTAKE(
      L3_CORAL.getValue().toBuilder()
          .intakeSpeed(IntakeSpeeds.coralOuttake)
          // Comment for autoformat
          .build()),

  // ==================== Algae Intake ====================
  // TODO: lollipop at some point?
  L2_ALGAE(
      SuperstructureStateData.builder()
          .elevatorHeight(0.0)
          .armAngle(0.62)
          .intakeSpeed(IntakeSpeeds.algaeActiveIntake)
          .build()),
  L2_ALGAE_UP(
      SuperstructureStateData.builder()
          .elevatorHeight(20.0)
          .armAngle(0.610)
          .intakeSpeed(IntakeSpeeds.algaeActiveIntake)
          .build()),

  L3_ALGAE(
      SuperstructureStateData.builder()
          .elevatorHeight(28.75)
          .armAngle(0.632)
          .intakeSpeed(IntakeSpeeds.algaeActiveIntake)
          .build()),
  L3_ALGAE_UP(
      SuperstructureStateData.builder()
          .elevatorHeight(44.5)
          .armAngle(0.63)
          .intakeSpeed(IntakeSpeeds.algaeActiveIntake)
          .build()),

  // ==================== Algae Outtake ====================
  PROCESSOR(
      SuperstructureStateData.builder()
          .elevatorHeight(0.0)
          .armAngle(0.54)
          .intakeSpeed(IntakeSpeeds.algaePassiveIntake)
          .build()),
  PROCESSOR_OUTTAKE(
      PROCESSOR.getValue().toBuilder()
          .intakeSpeed(IntakeSpeeds.algaeOuttake)
          // Comment for autoformat
          .build()),

  PRE_BARGE(
      SuperstructureStateData.builder()
          .elevatorHeight(44.5)
          .armAngle(0.85) // TODO: speed during transition
          .intakeSpeed(IntakeSpeeds.algaePassiveIntake)
          .groundIntakeAngle(0.05)
          .build()),
  BARGING(
      SuperstructureStateData.builder()
          .elevatorHeight(44.5)
          .armAngle(0.805)
          .intakeSpeed(IntakeSpeeds.algaePassiveIntake)
          .build()),
  BARGE(
      SuperstructureStateData.builder()
          .elevatorHeight(44.5)
          .armAngle(0.65)
          .intakeSpeed(IntakeSpeeds.algaeOuttake)
          .build()),

  // ==================== Climb ====================
  PRE_CLIMB(
      SuperstructureStateData.builder()
          .elevatorHeight(20.0)
          .armAngle(0.56)
          .intakeSpeed(0.0)
          .groundIntakeAngle(0.05)
          .groundIntakeFrontSpeed(0.0)
          .groundIntakeBackSpeed(0.0)
          .build()),
  CLIMB(
      SuperstructureStateData.builder()
          .elevatorHeight(20.0)
          .armAngle(0.56)
          .intakeSpeed(0.0)
          .groundIntakeAngle(0.31)
          .groundIntakeFrontSpeed(0.0)
          .groundIntakeBackSpeed(0.0)
          .build());

  public static final class IntakeSpeeds {
    // TODO: tune
    public static final double coralPassiveIntake = -0.3;
    public static final double coralActiveIntake = -1.0;
    public static final double coralOuttake = 1.0;

    public static final double algaePassiveIntake = 0.5;
    public static final double algaePassiveIntakeFast = 0.7; // For when arm flips around
    public static final double algaeActiveIntake = 1.0;
    public static final double algaeOuttake = -1.0;
  }

  private final SuperstructureStateData value;

  private SuperstructureState(SuperstructureStateData value) {
    this.value = value;
  }
}

package frc.robot.subsystems.superstructure;

import lombok.Getter;

@Getter
public enum SuperstructureState {
  IDLE(SuperstructureStateData.builder().build()),

  STOW(
      SuperstructureStateData.builder()
          .armAngle(Presets.Stow.armAngle)
          .elevatorHeight(Presets.Stow.elevatorHeight)
          .build()),

  GROUND_INTAKE_FOR_TRANSFER(SuperstructureStateData.builder().armAngle(0.959).build()),
  GROUND_INTAKE_FOR_L1(SuperstructureStateData.builder().armAngle(0.75).build()),

  GROUND_INTAKE_L1(SuperstructureStateData.builder().armAngle(0.75).build()),
  GROUND_INTAKE_L1_JERK(SuperstructureStateData.builder().armAngle(0.75).build()),

  L2_CORAL(SuperstructureStateData.builder().armAngle(0.610).build()),
  L2_CORAL_OUTTAKE(SuperstructureStateData.builder().build()),
  L3_CORAL(
      SuperstructureStateData.builder()
          .armAngle(0.63)
          .elevatorHeight(44.5)
          .groundIntakeAngle(0.31)
          .build()),
  L3_CORAL_OUTTAKE(L3_CORAL.getValue().toBuilder().intakeSpeed(0.0).build()),

  L2_ALGAE(SuperstructureStateData.builder().armAngle(0.62).build()),
  L3_ALGAE(
      SuperstructureStateData.builder()
          .armAngle(0.632)
          .elevatorHeight(28.75)
          .groundIntakeAngle(0.31)
          .groundIntakeFrontSpeed(0.0)
          .groundIntakeBackSpeed(0.0)
          .build()),
  ;

  private final SuperstructureStateData value;

  private SuperstructureState(SuperstructureStateData value) {
    this.value = value;
  }
}

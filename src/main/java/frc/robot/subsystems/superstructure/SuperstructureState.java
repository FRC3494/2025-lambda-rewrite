package frc.robot.subsystems.superstructure;

import lombok.Getter;

@Getter
public enum SuperstructureState {
  IDLE(SuperstructureStateData.builder().build()),

  GROUND_INTAKE_FOR_TRANSFER(SuperstructureStateData.builder().build()),
  GROUND_INTAKE_FOR_L1(SuperstructureStateData.builder().build()),

  L2_CORAL_OUTTAKE(SuperstructureStateData.builder().build()),
  L3_CORAL_OUTTAKE(SuperstructureStateData.builder().build()),
  ;

  private final SuperstructureStateData value;

  private SuperstructureState(SuperstructureStateData value) {
    this.value = value;
  }
}

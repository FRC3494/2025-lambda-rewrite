package frc.robot.subsystems.superstructure;

import lombok.Getter;

@Getter
public enum SuperstructureState {
  IDLE(SuperstructureStateData.builder().build());

  private final SuperstructureStateData value;

  private SuperstructureState(SuperstructureStateData value) {
    this.value = value;
  }
}

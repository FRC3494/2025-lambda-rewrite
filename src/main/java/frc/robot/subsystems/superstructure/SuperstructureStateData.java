package frc.robot.subsystems.superstructure;

import lombok.Builder;
import lombok.Getter;

@Builder(toBuilder = true)
@Getter
public class SuperstructureStateData {
  @Builder.Default public final Double elevatorHeight = null;

  @Builder.Default public final Double armAngle = null;
  @Builder.Default public final Double intakeSpeed = null;

  @Builder.Default public final Double groundIntakeAngle = null;
  @Builder.Default public final Double groundIntakeFrontSpeed = null;
  @Builder.Default public final Double groundIntakeBackSpeed = null;
}

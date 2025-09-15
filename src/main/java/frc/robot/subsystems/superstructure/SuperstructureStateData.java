package frc.robot.subsystems.superstructure;

import lombok.Builder;
import lombok.Getter;

@Builder(toBuilder = true)
@Getter
public class SuperstructureStateData {
  @Builder.Default public final double elevatorHeight = 0.0;

  public final double armAngle;
  @Builder.Default public final double intakeSpeed = 0.0;

  public final double groundIntakeAngle;
  @Builder.Default public final double groundIntakeFrontSpeed = 0.0;
  @Builder.Default public final double groundIntakeBackSpeed = 0.0;
}

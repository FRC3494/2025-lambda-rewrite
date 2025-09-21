package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private Spark leds;

  private LEDLightPattern pattern = LEDLightPattern.DISABLED;

  public LEDs() {
    leds = new Spark(Constants.LEDs.ledPwmId);
    leds.set(pattern.value);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LEDs/Pattern", pattern.name());
  }

  public Command setPattern(LEDLightPattern pattern) {
    return this.runOnce(
            () -> {
              this.pattern = pattern;
              leds.set(pattern.value);
            })
        .ignoringDisable(true);
  }

  public LEDLightPattern getPattern() {
    return pattern;
  }

  public static enum LEDLightPattern {
    DISABLED(Constants.LEDs.disabledColor),
    NONE(Constants.LEDs.noneColor),
    HAS_GAMEPIECE(Constants.LEDs.hasGamepieceColor),
    DEPOSITED(Constants.LEDs.depositedColor),
    INTAKING(Constants.LEDs.intakingColor);

    public final double value;

    private LEDLightPattern(double value) {
      this.value = value;
    }
  }
}

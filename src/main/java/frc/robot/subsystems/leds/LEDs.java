package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private Spark leds;

  private LEDPattern pattern = LEDPattern.DISABLED;

  public LEDs() {
    leds = new Spark(Constants.LEDs.ledPwmId);
    leds.set(pattern.value);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LEDs/Pattern", pattern.name());
  }

  public Command setPattern(LEDPattern pattern) {
    return this.runOnce(
            () -> {
              this.pattern = pattern;
              leds.set(pattern.value);
            })
        .ignoringDisable(true);
  }

  public static enum LEDPattern {
    DISABLED(Constants.LEDs.disabledColor),
    NONE(Constants.LEDs.noneColor),
    HAS_GAMEPIECE(Constants.LEDs.hasGamepieceColor),
    DEPOSITED(Constants.LEDs.depositedColor),
    INTAKING(Constants.LEDs.intakingColor);

    public final double value;

    private LEDPattern(double value) {
      this.value = value;
    }
  }
}

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.intake.Intake;

public class TeleopIntake extends Command {
  private Intake intake;
  private Superstructure superstructure;
  private LEDs leds;

  public TeleopIntake(Intake intake, Superstructure superstructure, LEDs leds) {
    this.intake = intake;
    this.superstructure = superstructure;
    this.leds = leds;

    addRequirements(intake);
    addRequirements(superstructure);
    addRequirements(leds);
  }

  @Override
  public void execute() {
    if (OI.getIntakeSpeed() != 0) {
      intake.setSpeed(OI.getIntakeSpeed()).schedule();

      if (OI.getIntakeSpeed() > 0
          && (Set.of(
                  SuperstructureState.ARM_L1_CORAL,
                  SuperstructureState.ARM_L1_CORAL_OUTTAKE,
                  SuperstructureState.L2_CORAL,
                  SuperstructureState.L2_CORAL_OUTTAKE,
                  SuperstructureState.L3_CORAL,
                  SuperstructureState.L3_CORAL_OUTTAKE)
              .contains(superstructure.getCurrentState()))) {
        leds.setPattern(LEDs.LEDLightPattern.DEPOSITED).schedule();
      }
    } else {
      superstructure.resumeCurrentState().schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.setSpeed(0.0);
  }
}

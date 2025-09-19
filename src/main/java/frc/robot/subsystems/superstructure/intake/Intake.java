package frc.robot.subsystems.superstructure.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private SparkMax intakeMotor;
  private SparkMaxConfig intakeMotorConfig;

  private double targetSpeed = 0.0;

  public Intake() {
    intakeMotor = new SparkMax(Constants.Intake.intakeMotorCanId, MotorType.kBrushless);
    intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig
        .idleMode(Constants.Intake.intakeMotorIdleMode)
        .inverted(Constants.Intake.intakeMotorInverted)
        .smartCurrentLimit(Constants.Intake.intakeMotorCurrentLimit);
    intakeMotor.configure(
        intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Speed", targetSpeed);
    Logger.recordOutput("Intake/Output", intakeMotor.getAppliedOutput());
    Logger.recordOutput("Intake/Current", intakeMotor.getOutputCurrent());
  }

  public Command setSpeed(Double speed) {
    return this.runOnce(
        () -> {
          if (speed != null) {
            targetSpeed = speed;
            intakeMotor.set(speed);
          }
        });
  }
}

package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  SparkMax climberMotor;

  private double targetPosition = 0.0;

  public Climber() {
    climberMotor = new SparkMax(Constants.Climber.climberMotorCanId, MotorType.kBrushless);
    SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
    climberMotorConfig
        .idleMode(Constants.Climber.climberMotorIdleMode)
        .inverted(Constants.Climber.climberMotorInverted)
        .smartCurrentLimit(Constants.Climber.climberMotorCurrentLimit);
    climberMotorConfig
        .closedLoop
        .pid(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD)
        .outputRange(-Constants.Climber.outputRange, Constants.Climber.outputRange)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    climberMotorConfig
        .closedLoop
        .maxMotion
        .maxVelocity(Constants.Climber.physicalMaxVelocity)
        .maxAcceleration(Constants.Climber.physicalMaxAcceleration)
        .allowedClosedLoopError(Constants.Climber.allowedError);
    climberMotor.configure(
        climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/TargetPosition", targetPosition);
    Logger.recordOutput("Climber/Position", getPosition());
    Logger.recordOutput("Climber/AtSetpoint", atSetpoint());
    Logger.recordOutput("Climber/Output", climberMotor.getAppliedOutput());
    Logger.recordOutput("Climber/Current", climberMotor.getOutputCurrent());
  }

  public Command setTargetPosition(Double position) {
    return this.runOnce(
        () -> {
          if (position != null) {
            targetPosition = position;
            climberMotor
                .getClosedLoopController()
                .setReference(position, ControlType.kMAXMotionPositionControl);
          }
        });
  }

  public double getPosition() {
    return climberMotor.getEncoder().getPosition();
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - targetPosition) < Constants.Climber.positionDeadband;
  }
}

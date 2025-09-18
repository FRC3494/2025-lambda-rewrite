package frc.robot.subsystems.superstructure.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private SparkFlex armMotor;
  private SparkFlexConfig armMotorConfig;

  private double targetPosition = 0.0;

  public Arm() {
    armMotor = new SparkFlex(Constants.Arm.armMotorCanId, MotorType.kBrushless);
    armMotorConfig = new SparkFlexConfig();
    armMotorConfig
        .idleMode(Constants.Arm.armMotorIdleMode)
        .inverted(Constants.Arm.armMotorInverted)
        .smartCurrentLimit(Constants.Arm.armMotorCurrentLimit);
    armMotorConfig
        .closedLoop
        .pid(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD)
        .outputRange(-Constants.Arm.outputRange, Constants.Arm.outputRange)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    armMotorConfig
        .closedLoop
        .maxMotion
        .maxVelocity(Constants.Arm.maxVelocity)
        .maxAcceleration(Constants.Arm.maxAcceleration)
        .allowedClosedLoopError(Constants.Arm.allowedError);
    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Arm/TargetPosition", targetPosition);
    Logger.recordOutput("Arm/Position", getPosition());
    Logger.recordOutput("Arm/Output", armMotor.getAppliedOutput());
    Logger.recordOutput("Arm/Current", armMotor.getOutputCurrent());
  }

  public void setTargetPosition(Double position) {
    if (position != null) {
      targetPosition = position;
      armMotor
          .getClosedLoopController()
          .setReference(position, ControlType.kMAXMotionPositionControl);
    }
  }

  public double getPosition() {
    return armMotor.getAbsoluteEncoder().getPosition();
  }
}

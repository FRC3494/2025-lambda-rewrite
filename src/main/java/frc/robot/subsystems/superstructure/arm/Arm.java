package frc.robot.subsystems.superstructure.arm;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private SparkFlex armMotor;

  private double targetPosition = 0.0;

  public Arm() {
    armMotor = new SparkFlex(Constants.Arm.armMotorCanId, MotorType.kBrushless);
    SparkFlexConfig armMotorConfig = new SparkFlexConfig();
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
        .maxVelocity(Constants.Arm.physicalMaxVelocity)
        .maxAcceleration(Constants.Arm.physicalMaxAcceleration)
        .allowedClosedLoopError(Constants.Arm.allowedError);
    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Arm/TargetPosition", targetPosition);
    Logger.recordOutput("Arm/Position", getPosition());
    Logger.recordOutput("Arm/AtSetpoint", atSetpoint());
    Logger.recordOutput("Arm/Output", armMotor.getAppliedOutput());
    Logger.recordOutput("Arm/Current", armMotor.getOutputCurrent());
  }

  public Command setTargetPosition(Double position) {
    return this.runOnce(
        () -> {
          if (position != null) {
            targetPosition = position;
            armMotor
                .getClosedLoopController()
                .setReference(position, ControlType.kMAXMotionPositionControl);
          }
        });
  }

  public double getPosition() {
    return armMotor.getAbsoluteEncoder().getPosition();
  }

  public boolean pastSafePosition() {
    // TODO: check if less than or greater than
    return getPosition() < Constants.Arm.safeAngle;
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - targetPosition) < Constants.Arm.positionDeadband;
  }
}

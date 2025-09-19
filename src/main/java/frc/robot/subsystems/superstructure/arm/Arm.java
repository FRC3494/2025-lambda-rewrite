package frc.robot.subsystems.superstructure.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.groundintake.GroundIntake;

public class Arm extends SubsystemBase {
  private static Arm instance = null;

  private SparkFlex armMotor;
  private SparkFlexConfig armMotorConfig;

  private double targetPosition = 0.0;

  private Arm() {
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

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Arm/TargetPosition", targetPosition);
    Logger.recordOutput("Arm/Position", getPosition());
    Logger.recordOutput("Arm/Output", armMotor.getAppliedOutput());
    Logger.recordOutput("Arm/Current", armMotor.getOutputCurrent());
  }

  public Command setTargetPosition(Double position) {
    return Commands.sequence(
        new WaitUntilCommand(
            () -> {
              // TODO: check if less than or greater than
              if (targetPosition < Constants.Arm.safeAngle) {
                return true;
              } else {
                return GroundIntake.getInstance().pastSafePosition();
              }
            }),
        this.runOnce(
            () -> {
              if (position != null) {
                targetPosition = position;
                armMotor
                    .getClosedLoopController()
                    .setReference(position, ControlType.kMAXMotionPositionControl);
              }
            }));
  }

  public double getPosition() {
    return armMotor.getAbsoluteEncoder().getPosition();
  }

  public boolean pastSafePosition() {
    // TODO: check if less than or greater than
    return getPosition() < Constants.Arm.safeAngle;
  }

  public boolean atTargetPosition() {
    return Math.abs(getPosition() - targetPosition) < Constants.Arm.positionDeadband;
  }
}

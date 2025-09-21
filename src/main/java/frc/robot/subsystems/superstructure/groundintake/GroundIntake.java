package frc.robot.subsystems.superstructure.groundintake;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase {
  private SparkFlex pivotMotor;
  private double targetPivotPosition = 0.0;

  private SparkMax frontIntakeMotor;
  private double targetFrontIntakeSpeed = 0.0;

  private SparkMax backIntakeMotor;
  private double targetBackIntakeSpeed = 0.0;

  private TimeOfFlight distanceSensor;

  public GroundIntake() {
    pivotMotor = new SparkFlex(Constants.GroundIntake.pivotMotorCanId, MotorType.kBrushless);
    SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    pivotMotorConfig
        .idleMode(Constants.GroundIntake.pivotMotorIdleMode)
        .inverted(Constants.GroundIntake.pivotMotorInverted)
        .smartCurrentLimit(Constants.GroundIntake.pivotMotorCurrentLimit);
    pivotMotorConfig
        .closedLoop
        .pid(
            Constants.GroundIntake.pivotkP,
            Constants.GroundIntake.pivotkI,
            Constants.GroundIntake.pivotkD)
        .outputRange(
            -Constants.GroundIntake.pivotOutputRange, Constants.GroundIntake.pivotOutputRange)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pivotMotorConfig
        .closedLoop
        .maxMotion
        .maxVelocity(Constants.GroundIntake.physicalPivotMaxVelocity)
        .maxAcceleration(Constants.GroundIntake.physicalPivotMaxAcceleration)
        .allowedClosedLoopError(Constants.GroundIntake.pivotAllowedError);
    pivotMotor.configure(
        pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    frontIntakeMotor =
        new SparkMax(Constants.GroundIntake.frontIntakeMotorCanId, MotorType.kBrushless);
    SparkMaxConfig frontIntakeMotorConfig = new SparkMaxConfig();
    frontIntakeMotorConfig
        .idleMode(Constants.GroundIntake.intakeMotorIdleMode)
        .inverted(Constants.GroundIntake.frontIntakeMotorInverted)
        .smartCurrentLimit(Constants.GroundIntake.intakeMotorCurrentLimit);
    frontIntakeMotor.configure(
        frontIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    backIntakeMotor =
        new SparkMax(Constants.GroundIntake.backIntakeMotorCanId, MotorType.kBrushless);
    SparkMaxConfig backIntakeMotorConfig = new SparkMaxConfig();
    backIntakeMotorConfig
        .idleMode(Constants.GroundIntake.intakeMotorIdleMode)
        .inverted(Constants.GroundIntake.backIntakeMotorInverted)
        .smartCurrentLimit(Constants.GroundIntake.intakeMotorCurrentLimit);
    backIntakeMotor.configure(
        backIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    distanceSensor = new TimeOfFlight(Constants.GroundIntake.distanceSensorDeviceNumber);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("GroundIntake/TargetPivotPosition", targetPivotPosition);
    Logger.recordOutput("GroundIntake/PivotPosition", getPivotPosition());
    Logger.recordOutput("GroundIntake/AtSetpoint", atSetpoint());
    Logger.recordOutput("GroundIntake/PivotOutput", pivotMotor.getAppliedOutput());
    Logger.recordOutput("GroundIntake/PivotCurrent", pivotMotor.getOutputCurrent());

    Logger.recordOutput("GroundIntake/FrontIntakeSpeed", targetFrontIntakeSpeed);
    Logger.recordOutput("GroundIntake/FrontIntakeOutput", frontIntakeMotor.getAppliedOutput());
    Logger.recordOutput("GroundIntake/FrontIntakeCurrent", frontIntakeMotor.getOutputCurrent());

    Logger.recordOutput("GroundIntake/BackIntakeSpeed", targetBackIntakeSpeed);
    Logger.recordOutput("GroundIntake/BackIntakeOutput", backIntakeMotor.getAppliedOutput());
    Logger.recordOutput("GroundIntake/BackIntakeCurrent", backIntakeMotor.getOutputCurrent());

    Logger.recordOutput("GroundIntake/DistanceSensor", distanceSensor.getRange());
    Logger.recordOutput("GroundIntake/DistanceSensorTripped", distanceSensorTripped());
  }

  public Command setTargetPivotPosition(Double position) {
    return this.runOnce(
        () -> {
          if (position != null) {
            targetPivotPosition = position;
            pivotMotor
                .getClosedLoopController()
                .setReference(position, ControlType.kMAXMotionPositionControl);
          }
        });
  }

  public Command setIntakeSpeeds(Double frontSpeed, Double backSpeed) {
    return this.runOnce(
        () -> {
          if (frontSpeed != null) {
            targetFrontIntakeSpeed = frontSpeed;
            frontIntakeMotor.set(frontSpeed);
          }
          if (backSpeed != null) {
            targetBackIntakeSpeed = backSpeed;
            backIntakeMotor.set(backSpeed);
          }
        });
  }

  public boolean distanceSensorTripped() {
    return distanceSensor.getRange() < Constants.GroundIntake.distanceSensorCoralThreshold;
  }

  public double getPivotPosition() {
    return pivotMotor.getAbsoluteEncoder().getPosition();
  }

  public boolean pastSafePosition() {
    return getPivotPosition() < Constants.GroundIntake.safeAngle;
  }

  public boolean atSetpoint() {
    return Math.abs(getPivotPosition() - targetPivotPosition)
        < Constants.GroundIntake.positionDeadband;
  }
}

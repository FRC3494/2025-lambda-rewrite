package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax leaderMotor;
  private SparkMaxConfig leaderMotorConfig;

  private SparkMax followerMotor;
  private SparkMaxConfig followerMotorConfig;

  private double targetPosition = 0.0;

  private DigitalInput bottomMagSensor;

  public Elevator() {
    leaderMotor = new SparkMax(Constants.Elevator.leaderMotorCanId, MotorType.kBrushless);
    leaderMotorConfig = new SparkMaxConfig();
    leaderMotorConfig
        .idleMode(Constants.Elevator.motorIdleMode)
        .inverted(Constants.Elevator.leaderMotorInverted)
        .smartCurrentLimit(Constants.Elevator.motorCurrentLimit);
    leaderMotorConfig
        .closedLoop
        .pid(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD)
        .outputRange(-Constants.Elevator.outputRange, Constants.Elevator.outputRange);
    leaderMotorConfig
        .closedLoop
        .maxMotion
        .maxVelocity(Constants.Elevator.maxVelocity)
        .maxAcceleration(Constants.Elevator.maxAcceleration)
        .allowedClosedLoopError(Constants.Elevator.allowedError);

    followerMotor = new SparkMax(Constants.Elevator.followerMotorCanId, MotorType.kBrushless);
    followerMotorConfig = new SparkMaxConfig();
    followerMotorConfig
        .idleMode(Constants.Elevator.motorIdleMode)
        .inverted(Constants.Elevator.followerMotorInverted)
        .smartCurrentLimit(Constants.Elevator.motorCurrentLimit);
    followerMotorConfig.follow(leaderMotor);

    leaderMotor.configure(
        leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(
        followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    bottomMagSensor = new DigitalInput(Constants.Elevator.bottomMagSensorDioChannel);
  }

  @Override
  public void periodic() {
    if (bottomSensorTripped()) {
      rezeroPosition();
    }

    Logger.recordOutput("Elevator/TargetPosition", targetPosition);
    Logger.recordOutput("Elevator/LeaderPosition", leaderMotor.getEncoder().getPosition());
    Logger.recordOutput("Elevator/FollowerPosition", followerMotor.getEncoder().getPosition());
    Logger.recordOutput("Elevator/LeaderOutput", leaderMotor.getAppliedOutput());
    Logger.recordOutput("Elevator/FollowerOutput", followerMotor.getAppliedOutput());
    Logger.recordOutput("Elevator/LeaderCurrent", leaderMotor.getOutputCurrent());
    Logger.recordOutput("Elevator/FollowerCurrent", followerMotor.getOutputCurrent());

    Logger.recordOutput("Elevator/BottomSensorTripped", bottomSensorTripped());
  }

  public void setTargetPosition(double position) {
    targetPosition = position;
    leaderMotor
        .getClosedLoopController()
        .setReference(position, ControlType.kMAXMotionPositionControl);
  }

  public void rezeroPosition() {
    leaderMotor.getEncoder().setPosition(0.0);
    followerMotor.getEncoder().setPosition(0.0);
  }

  public boolean bottomSensorTripped() {
    return bottomMagSensor.get();
  }
}

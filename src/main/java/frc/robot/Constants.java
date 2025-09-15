// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Drive {
    public static final double maxSpeedMetersPerSec = Units.feetToMeters(14.5);
    public static final double odometryFrequency =
        100.0; // Hz //TODO: research what this actually is
    public static final double trackWidth = Units.inchesToMeters(20.75);
    public static final double wheelBase = Units.inchesToMeters(20.75);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(Math.toRadians(201.9));
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(Math.toRadians(239.4));
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.toRadians(265.0));
    public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.toRadians(147.9));

    // Device CAN IDs
    public static final int pigeonCanId = 52;

    public static final int frontLeftDriveCanId = 18;
    public static final int frontRightDriveCanId = 19;
    public static final int backLeftDriveCanId = 30;
    public static final int backRightDriveCanId = 1;

    public static final int frontLeftTurnCanId = 16;
    public static final int frontRightTurnCanId = 17;
    public static final int backLeftTurnCanId = 2;
    public static final int backRightTurnCanId = 3;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 50; // TODO: slip current test
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.6845632679942868);
    public static final double driveMotorReduction =
        (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0); // L1 Gearing //TODO: might be L2? I forgot
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
    // Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
    // Wheel Rad/Sec

    // Drive PID configuration //TODO: tune
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration //TODO: verify
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration //TODO: verify
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration //TODO: tune
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration //TODO: get info
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig =
        new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec,
                wheelCOF,
                driveGearbox.withReduction(driveMotorReduction),
                driveMotorCurrentLimit,
                1),
            moduleTranslations);
  }

  public static class Elevator {
    public static final int bottomMagSensorDioChannel = 9;

    public static final int leaderMotorCanId = 12;
    public static final boolean leaderMotorInverted = true;
    public static final int followerMotorCanId = 13;
    public static final boolean followerMotorInverted = false;

    public static final double bottomHeight = Units.inchesToMeters(0.0); // TODO: get info
    public static final double drumRadius = Units.inchesToMeters(0.0);

    // TODO: tune
    public static final int motorCurrentLimit = 80;
    public static final IdleMode motorIdleMode = IdleMode.kBrake;

    // TODO: tune
    public static final double kP = 0.8;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double outputRange = 1.0;
    public static final double maxVelocity = 0.0;
    public static final double maxAcceleration = 0.0;
    public static final double allowedError = 0.0;
  }

  public static class Arm {
    public static final int armMotorCanId = 15;
    public static final boolean armMotorInverted = false;
    public static final IdleMode armMotorIdleMode = IdleMode.kBrake;
    public static final int armMotorCurrentLimit = 75;

    // TODO: tune
    public static final double kP = 9.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double outputRange = 1.0;
    public static final double maxVelocity = 0.0;
    public static final double maxAcceleration = 0.0;
    public static final double allowedError = 0.0;
  }

  public static class Intake {
    public static final int intakeMotorCanId = 14;
    public static final boolean intakeMotorInverted = false;
    public static final IdleMode intakeMotorIdleMode = IdleMode.kBrake;
    public static final int intakeMotorCurrentLimit = 30;
  }

  public static class LEDs {
    public static int ledPwmId = 0;

    // Color values: https://docs.revrobotics.com/rev-crossover-products/blinkin/gs/patterns
    public static double disabledColor = 0.0;
    public static double noneColor = 0.65; // Solid Orange
    public static double hasGamepieceColor = 0.75; // Solid Dark Green
    public static double depositedColor = 0.69; // Solid Yellow
    public static double intakingColor = 0.59; // Solid Dark Red
  }
}

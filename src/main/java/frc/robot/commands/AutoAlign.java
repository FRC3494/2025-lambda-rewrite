package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlign.ReefAlgaeLocation;
import frc.robot.Constants.AutoAlign.ReefCoralLocation;
import frc.robot.subsystems.drive.Drive;

public class AutoAlign extends Command {
  private final Drive drive;

  private final ProfiledPIDController linearController;
  private final ProfiledPIDController angularController;

  private final Supplier<Pose2d> getTargetPose;
  private Pose2d targetPose;

  public AutoAlign(Drive drive, Supplier<Pose2d> getTargetPoseBlue) {
    this.drive = drive;
    addRequirements(drive);

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      this.getTargetPose = getTargetPoseBlue;
    } else {
      this.getTargetPose = () -> blueToRed(getTargetPoseBlue.get());
    }

    linearController =
        new ProfiledPIDController(
            Constants.AutoAlign.linearP,
            Constants.AutoAlign.linearI,
            Constants.AutoAlign.linearD,
            new TrapezoidProfile.Constraints(
                Constants.AutoAlign.linearMaxVelocity, Constants.AutoAlign.linearMaxAcceleration));
    linearController.setTolerance(Constants.AutoAlign.linearTolerance);

    angularController =
        new ProfiledPIDController(
            Constants.AutoAlign.angularP,
            Constants.AutoAlign.angularI,
            Constants.AutoAlign.angularD,
            new TrapezoidProfile.Constraints(
                Constants.AutoAlign.angularMaxVelocity,
                Constants.AutoAlign.angularMaxAcceleration));
    angularController.enableContinuousInput(-Math.PI, Math.PI);
    angularController.setTolerance(Constants.AutoAlign.angularTolerance);
  }

  @Override
  public void initialize() {
    resetPIDControllers();
  }

  @Override
  public void execute() {
    if (!getTargetPose.get().equals(targetPose)) {
      resetPIDControllers();
    }

    double linearSpeed =
        linearController.calculate(
            drive.getPose().getTranslation().getDistance(targetPose.getTranslation()), 0);
    double angleToGoal =
        Math.tan(
            (targetPose.getY() - drive.getPose().getY())
                / (targetPose.getX() - drive.getPose().getX()));

    double angularSpeed = angularController.calculate(drive.getPose().getRotation().getRadians());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                linearSpeed * Math.cos(angleToGoal),
                linearSpeed * Math.sin(angleToGoal),
                angularSpeed),
            drive.getRotation()));

    Logger.recordOutput("AutoAlign/CurrentPose", drive.getPose());
    Logger.recordOutput("AutoAlign/GoalPose", targetPose);
    Logger.recordOutput("AutoAlign/LinearError", linearController.getPositionError());
    Logger.recordOutput(
        "AutoAlign/LinearSetpointPosition", linearController.getSetpoint().position);
    Logger.recordOutput(
        "AutoAlign/LinearSetpointVelocity", linearController.getSetpoint().velocity);
    Logger.recordOutput("AutoAlign/AngularError", angularController.getPositionError());
    Logger.recordOutput(
        "AutoAlign/AngularSetpointPosition", angularController.getSetpoint().position);
    Logger.recordOutput(
        "AutoAlign/AngularSetpointVelocity", angularController.getSetpoint().velocity);
  }

  private void resetPIDControllers() {
    targetPose = getTargetPose.get();

    linearController.reset(
        drive.getPose().getTranslation().getDistance(targetPose.getTranslation()));
    linearController.setGoal(0.0);

    angularController.reset(drive.getPose().getRotation().getRadians());
    angularController.setGoal(targetPose.getRotation().getRadians());
  }

  @Override
  public boolean isFinished() {
    return linearController.atGoal() && angularController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  public static Pose2d getClosestReefLocationBlue(Pose2d robotPose, boolean left, boolean right) {
    double minDistance = Double.MAX_VALUE;
    ReefAlgaeLocation closestFace = null;

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.get() == Alliance.Blue) {
      for (ReefAlgaeLocation location : Constants.AutoAlign.ReefAlgaeLocation.values()) {
        double distance = robotPose.getTranslation().getDistance(location.pose.getTranslation());
        if (distance < minDistance) {
          minDistance = distance;
          closestFace = location;
        }
      }
    } else {
      for (ReefAlgaeLocation location : Constants.AutoAlign.ReefAlgaeLocation.values()) {
        double distance =
            robotPose.getTranslation().getDistance(blueToRed(location.pose).getTranslation());
        if (distance < minDistance) {
          minDistance = distance;
          closestFace = location;
        }
      }
    }

    if (left && right) {
      return closestFace.pose;
    } else if (left) {
      switch (closestFace) {
        case AB:
          return ReefCoralLocation.A.pose;
        case CD:
          return ReefCoralLocation.C.pose;
        case EF:
          return ReefCoralLocation.E.pose;
        case GH:
          return ReefCoralLocation.G.pose;
        case IJ:
          return ReefCoralLocation.I.pose;
        case KL:
          return ReefCoralLocation.K.pose;
        default:
          return closestFace.pose;
      }
    } else if (right) {
      switch (closestFace) {
        case AB:
          return ReefCoralLocation.B.pose;
        case CD:
          return ReefCoralLocation.D.pose;
        case EF:
          return ReefCoralLocation.F.pose;
        case GH:
          return ReefCoralLocation.H.pose;
        case IJ:
          return ReefCoralLocation.J.pose;
        case KL:
          return ReefCoralLocation.L.pose;
        default:
          return closestFace.pose;
      }
    } else {
      return closestFace.pose;
    }
  }

  public static Pose2d blueToRed(Pose2d bluePose) {
    return new Pose2d(
        Constants.Vision.aprilTagLayout.getFieldWidth() - bluePose.getX(),
        Constants.Vision.aprilTagLayout.getFieldLength() - bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.fromDegrees(180)));
  }
}

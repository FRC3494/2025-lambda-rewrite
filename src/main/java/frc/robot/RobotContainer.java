// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopIntake;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.LEDLightPattern;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.groundintake.GroundIntake;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision aprilTagVision;

  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  private final GroundIntake groundIntake;
  private final Superstructure superstructure;

  private final Climber climber;

  final LEDs leds;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  // @CodeScene(disable: "Large Method")
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        {
          // Real robot, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSpark(0),
                  new ModuleIOSpark(1),
                  new ModuleIOSpark(2),
                  new ModuleIOSpark(3));
          aprilTagVision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOLimelight(Constants.Vision.limelightRightName, drive::getRotation),
                  new VisionIOLimelight(Constants.Vision.limelightLeftName, drive::getRotation),
                  new VisionIOLimelight(Constants.Vision.limelightSwerveName, drive::getRotation),
                  new VisionIOLimelight(Constants.Vision.limelightBargeName, drive::getRotation));
          break;
        }

      case SIM:
        {
          // Sim robot, instantiate physics sim IO implementations
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          aprilTagVision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {});
          break;
        }

      default:
        {
          // Replayed robot, disable IO implementations
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
          aprilTagVision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {});
          break;
        }
    }

    elevator = new Elevator();
    arm = new Arm();
    intake = new Intake();
    groundIntake = new GroundIntake();
    superstructure = new Superstructure(elevator, arm, intake, groundIntake);

    climber = new Climber();

    leds = new LEDs();

    intake.setDefaultCommand(new TeleopIntake(intake, superstructure, leds));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  // @CodeScene(disable: "Large Method")
  private void configureButtonBindings() {

    // * ================ Drive ================

    Command normalDriveCommand =
        DriveCommands.joystickDrive(drive, OI.Drive::getX, OI.Drive::getY, OI.Drive::getOmega);

    // Default command, normal field-relative drive
    drive.setDefaultCommand(normalDriveCommand);

    // Switch to X pattern when X button is pressed
    OI.Drive.stopWithX().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    OI.Drive.rezeroGyro()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // ==================== Auto-Align ====================
    OI.AutoAlign.coral()
        .onTrue(
            Commands.runOnce(() -> drive.setDefaultCommand(DriveCommands.coralAutoAlign(drive))))
        .onFalse(Commands.runOnce(() -> drive.setDefaultCommand(normalDriveCommand)));

    OI.AutoAlign.reefLeft()
        .or(OI.AutoAlign.reefRight())
        .onTrue(
            Commands.runOnce(
                () ->
                    drive.setDefaultCommand(
                        new AutoAlign(
                            drive,
                            () ->
                                AutoAlign.getClosestReefLocationBlue(
                                    drive.getPose(),
                                    OI.AutoAlign.reefLeft().getAsBoolean(),
                                    OI.AutoAlign.reefRight().getAsBoolean())))))
        .onFalse(Commands.runOnce(() -> drive.setDefaultCommand(normalDriveCommand)));

    OI.AutoAlign.barge()
        .onTrue(
            Commands.runOnce(
                () ->
                    drive.setDefaultCommand(
                        new AutoAlign(drive, () -> Constants.AutoAlign.bargeLocation))))
        .onFalse(Commands.runOnce(() -> drive.setDefaultCommand(normalDriveCommand)));

    // TODO: toggle defense sensor?

    // ==================== Stow ====================
    OI.stow().onTrue(superstructure.setCurrentTargetState(SuperstructureState.STOW));

    // ==================== Ground Intake ====================
    OI.groundIntakeForTransfer()
        .onTrue(
            Commands.sequence(
                leds.setPattern(LEDLightPattern.INTAKING),
                superstructure.setCurrentTargetState(
                    SuperstructureState.GROUND_INTAKE_FOR_TRANSFER),
                new WaitUntilCommand(groundIntake::distanceSensorTripped),
                leds.setPattern(LEDLightPattern.HAS_GAMEPIECE)
                    .onlyIf(OI.groundIntakeForTransfer()::getAsBoolean)))
        .onFalse(
            Commands.sequence(
                superstructure
                    .setCurrentTargetState(SuperstructureState.DONE_WITH_GROUND_INTAKE_FOR_TRANSFER)
                    .onlyIf(OI.getPrimaryController()::isConnected),
                leds.setPattern(LEDLightPattern.NONE)
                    .onlyIf(() -> leds.getPattern() != LEDLightPattern.HAS_GAMEPIECE)));

    OI.groundIntakeForL1()
        .onTrue(
            Commands.sequence(
                leds.setPattern(LEDLightPattern.INTAKING),
                superstructure.setCurrentTargetState(SuperstructureState.GROUND_INTAKE_FOR_L1),
                new WaitUntilCommand(groundIntake::distanceSensorTripped),
                leds.setPattern(LEDLightPattern.HAS_GAMEPIECE)
                    .onlyIf(OI.groundIntakeForL1()::getAsBoolean)))
        .onFalse(
            Commands.sequence(
                superstructure
                    .setCurrentTargetState(SuperstructureState.GROUND_INTAKE_L1)
                    .onlyIf(OI.getPrimaryController()::isConnected),
                leds.setPattern(LEDLightPattern.NONE)
                    .onlyIf(() -> leds.getPattern() != LEDLightPattern.HAS_GAMEPIECE)));

    OI.groundIntakeL1Outtake()
        .onTrue(
            Commands.sequence(
                superstructure.setCurrentTargetState(SuperstructureState.GROUND_INTAKE_L1_JERK),
                new WaitUntilCommand(() -> !groundIntake.distanceSensorTripped()),
                leds.setPattern(LEDLightPattern.DEPOSITED)))
        .onFalse(
            Commands.sequence(
                superstructure
                    .setCurrentTargetState(SuperstructureState.GROUND_INTAKE_L1)
                    .onlyIf(OI.getLeftButtonBoard()::isConnected),
                leds.setPattern(LEDLightPattern.NONE)));

    OI.groundIntakeManualOuttake()
        .onTrue(groundIntake.setIntakeSpeeds(0.2, -0.5))
        .onFalse(
            Commands.sequence(
                superstructure.resumeCurrentState(),
                // Comment for autoformat
                leds.setPattern(LEDLightPattern.NONE)));

    // ==================== Coral Intake w/ Arm ====================
    OI.feeder().onTrue(superstructure.setCurrentTargetState(SuperstructureState.FEEDER));

    // ==================== Coral Outtake w/ Arm ====================
    OI.armL1Coral().onTrue(superstructure.setCurrentTargetState(SuperstructureState.ARM_L1_CORAL));

    OI.L2Coral().onTrue(superstructure.setCurrentTargetState(SuperstructureState.L2_CORAL));

    OI.L3Coral().onTrue(superstructure.setCurrentTargetState(SuperstructureState.L3_CORAL));

    // ==================== Algae Intake ====================
    OI.L2Algae()
        .onTrue(superstructure.setCurrentTargetState(SuperstructureState.L2_ALGAE))
        .onFalse(superstructure.setCurrentTargetState(SuperstructureState.L2_ALGAE_UP));

    OI.L3Algae()
        .onTrue(superstructure.setCurrentTargetState(SuperstructureState.L3_ALGAE))
        .onFalse(superstructure.setCurrentTargetState(SuperstructureState.L3_ALGAE_UP));

    // ==================== Algae Outtake ====================
    OI.processor().onTrue(superstructure.setCurrentTargetState(SuperstructureState.PROCESSOR));

    OI.preBarge().onTrue(superstructure.setCurrentTargetState(SuperstructureState.PRE_BARGE));

    OI.barge().onTrue(superstructure.setCurrentTargetState(SuperstructureState.BARGE));

    // ==================== Climb ====================
    OI.preClimb()
        .onTrue(
            Commands.sequence(
                superstructure.setCurrentTargetState(SuperstructureState.PRE_CLIMB),
                climber.setTargetPosition(Constants.Climber.preClimbAngle)));

    OI.climbStage1().onTrue(climber.setTargetPosition(Constants.Climber.stage1Angle));

    OI.climbStage2().onTrue(climber.setTargetPosition(Constants.Climber.stage2Angle));

    OI.climbStage3().onTrue(superstructure.setCurrentTargetState(SuperstructureState.CLIMB));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

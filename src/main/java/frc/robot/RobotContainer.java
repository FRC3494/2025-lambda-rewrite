// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.TeleopIntake;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.groundintake.GroundIntake;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  private final Superstructure superstructure;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  private final GroundIntake groundIntake;

  private final Climber climber;

  final LEDs leds;

  private final RobotCommands robotCommands;

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

    robotCommands =
        new RobotCommands(
            drive, superstructure, elevator, arm, intake, groundIntake, climber, leds);

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
    OI.stow().onTrue(robotCommands.stow());

    // ==================== Ground Intake ====================
    OI.groundIntakeForTransfer()
        .onTrue(robotCommands.groundIntakeForTransferRising())
        .onFalse(robotCommands.groundIntakeForTransferFalling());

    OI.groundIntakeForL1()
        .onTrue(robotCommands.groundIntakeForL1Rising())
        .onFalse(robotCommands.groundIntakeForL1Falling());

    OI.groundIntakeL1Outtake()
        .onTrue(robotCommands.groundIntakeL1OuttakeRising())
        .onFalse(robotCommands.groundIntakeL1OuttakeFalling());

    OI.groundIntakeManualOuttake()
        .onTrue(robotCommands.groundIntakeManualOuttakeRising())
        .onFalse(robotCommands.groundIntakeManualOuttakeFalling());

    // ==================== Coral Intake w/ Arm ====================
    OI.feeder().onTrue(robotCommands.feeder());

    // ==================== Coral Outtake w/ Arm ====================
    OI.armL1Coral().onTrue(robotCommands.armL1Coral());

    OI.L2Coral().onTrue(robotCommands.L2Coral());

    OI.L3Coral().onTrue(robotCommands.L3Coral());

    // ==================== Algae Intake ====================
    OI.L2Algae()
        // Comment for autoformat
        .onTrue(robotCommands.L2AlgaeRising())
        .onFalse(robotCommands.L2AlgaeFalling());

    OI.L3Algae()
        // Comment for autoformat
        .onTrue(robotCommands.L3AlgaeRising())
        .onFalse(robotCommands.L3AlgaeFalling());

    // ==================== Algae Outtake ====================
    OI.processor().onTrue(robotCommands.processor());

    OI.preBarge().onTrue(robotCommands.preBarge());

    OI.barge().onTrue(robotCommands.barge());

    // ==================== Climb ====================
    OI.preClimb().onTrue(robotCommands.preClimb());

    OI.climbStage1().onTrue(robotCommands.climbStage1());

    OI.climbStage2().onTrue(robotCommands.climbStage2());

    OI.climbStage3().onTrue(robotCommands.climbStage3());
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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.LEDLightPattern;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.groundintake.GroundIntake;
import frc.robot.subsystems.superstructure.intake.Intake;

public class RobotCommands {
  private final Drive drive;

  private final Superstructure superstructure;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  private final GroundIntake groundIntake;

  private final Climber climber;

  private final LEDs leds;

  public RobotCommands(
      Drive drive,
      Superstructure superstructure,
      Elevator elevator,
      Arm arm,
      Intake intake,
      GroundIntake groundIntake,
      Climber climber,
      LEDs leds) {
    this.drive = drive;

    this.superstructure = superstructure;
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.groundIntake = groundIntake;

    this.climber = climber;

    this.leds = leds;
  }

  // ==================== Stow ====================
  public Command stow() {
    return superstructure.setCurrentTargetState(SuperstructureState.STOW);
  }

  // ==================== Ground Intake ====================
  public Command groundIntakeForTransferRising() {
    return Commands.sequence(
        leds.setPattern(LEDLightPattern.INTAKING),
        superstructure.setCurrentTargetState(SuperstructureState.GROUND_INTAKE_FOR_TRANSFER),
        new WaitUntilCommand(groundIntake::distanceSensorTripped),
        leds.setPattern(LEDLightPattern.HAS_GAMEPIECE)
            .onlyIf(OI.groundIntakeForTransfer()::getAsBoolean));
  }

  public Command groundIntakeForTransferFalling() {
    return Commands.sequence(
        superstructure
            .setCurrentTargetState(SuperstructureState.DONE_WITH_GROUND_INTAKE_FOR_TRANSFER)
            .onlyIf(OI.getPrimaryController()::isConnected),
        leds.setPattern(LEDLightPattern.NONE)
            .onlyIf(() -> leds.getPattern() != LEDLightPattern.HAS_GAMEPIECE));
  }

  public Command groundIntakeForL1Rising() {
    return Commands.sequence(
        leds.setPattern(LEDLightPattern.INTAKING),
        superstructure.setCurrentTargetState(SuperstructureState.GROUND_INTAKE_FOR_L1),
        new WaitUntilCommand(groundIntake::distanceSensorTripped),
        leds.setPattern(LEDLightPattern.HAS_GAMEPIECE)
            .onlyIf(OI.groundIntakeForL1()::getAsBoolean));
  }

  public Command groundIntakeForL1Falling() {
    return Commands.sequence(
        superstructure
            .setCurrentTargetState(SuperstructureState.GROUND_INTAKE_L1)
            .onlyIf(OI.getPrimaryController()::isConnected),
        leds.setPattern(LEDLightPattern.NONE)
            .onlyIf(() -> leds.getPattern() != LEDLightPattern.HAS_GAMEPIECE));
  }

  public Command groundIntakeL1OuttakeRising() {
    return Commands.sequence(
        superstructure.setCurrentTargetState(SuperstructureState.GROUND_INTAKE_L1_JERK),
        new WaitUntilCommand(() -> !groundIntake.distanceSensorTripped()),
        leds.setPattern(LEDLightPattern.DEPOSITED));
  }

  public Command groundIntakeL1OuttakeFalling() {
    return Commands.sequence(
        superstructure
            .setCurrentTargetState(SuperstructureState.GROUND_INTAKE_L1)
            .onlyIf(OI.getLeftButtonBoard()::isConnected),
        leds.setPattern(LEDLightPattern.NONE));
  }

  public Command groundIntakeManualOuttakeRising() {
    return groundIntake.setIntakeSpeeds(0.2, -0.5);
  }

  public Command groundIntakeManualOuttakeFalling() {
    return Commands.sequence(
        superstructure.resumeCurrentState(),
        // Comment for autoformat
        leds.setPattern(LEDLightPattern.NONE));
  }

  // ==================== Coral Intake w/ Arm ====================
  public Command feeder() {
    return superstructure.setCurrentTargetState(SuperstructureState.FEEDER);
  }

  // ==================== Coral Outtake w/ Arm ====================
  public Command armL1Coral() {
    return superstructure.setCurrentTargetState(SuperstructureState.ARM_L1_CORAL);
  }

  public Command L2Coral() {
    return superstructure.setCurrentTargetState(SuperstructureState.L2_CORAL);
  }

  public Command L3Coral() {
    return superstructure.setCurrentTargetState(SuperstructureState.L3_CORAL);
  }

  // ==================== Algae Intake ====================
  public Command L2AlgaeRising() {
    return superstructure.setCurrentTargetState(SuperstructureState.L2_ALGAE);
  }

  public Command L2AlgaeFalling() {
    return superstructure.setCurrentTargetState(SuperstructureState.L2_ALGAE_UP);
  }

  public Command L3AlgaeRising() {
    return superstructure.setCurrentTargetState(SuperstructureState.L3_ALGAE);
  }

  public Command L3AlgaeFalling() {
    return superstructure.setCurrentTargetState(SuperstructureState.L3_ALGAE_UP);
  }

  // ==================== Algae Outtake ====================
  public Command processor() {
    return superstructure.setCurrentTargetState(SuperstructureState.PROCESSOR);
  }

  public Command preBarge() {
    return superstructure.setCurrentTargetState(SuperstructureState.PRE_BARGE);
  }

  public Command barge() {
    return superstructure.setCurrentTargetState(SuperstructureState.BARGE);
  }

  // ==================== Climb ====================
  public Command preClimb() {
    return Commands.sequence(
        superstructure.setCurrentTargetState(SuperstructureState.PRE_CLIMB),
        climber.setTargetPosition(Constants.Climber.preClimbAngle));
  }

  public Command climbStage1() {
    return climber.setTargetPosition(Constants.Climber.stage1Angle);
  }

  public Command climbStage2() {
    return climber.setTargetPosition(Constants.Climber.stage2Angle);
  }

  public Command climbStage3() {
    return superstructure.setCurrentTargetState(SuperstructureState.CLIMB);
  }
}

package frc.robot.subsystems.superstructure;

import lombok.Getter;

@Getter
public enum SuperstructureState {
  IDLE(SuperstructureStateData.builder().build()),

  STOW(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.Stow.elevatorHeight)
          .armAngle(Presets.Stow.armAngle)
          .intakeSpeed(Presets.Stow.intakeSpeed)
          .groundIntakeAngle(Presets.Stow.groundIntakeAngle)
          .groundIntakeFrontSpeed(Presets.Stow.groundIntakeFrontSpeed)
          .groundIntakeBackSpeed(Presets.Stow.groundIntakeBackSpeed)
          .build()),

  GROUND_INTAKE_FOR_TRANSFER(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.GroundIntakeForTransfer.elevatorHeight)
          .armAngle(Presets.GroundIntakeForTransfer.armAngle)
          .intakeSpeed(Presets.GroundIntakeForTransfer.intakeSpeed)
          .groundIntakeAngle(Presets.GroundIntakeForTransfer.groundIntakeAngle)
          .groundIntakeFrontSpeed(Presets.GroundIntakeForTransfer.groundIntakeFrontSpeed)
          .groundIntakeBackSpeed(Presets.GroundIntakeForTransfer.groundIntakeBackSpeed)
          .build()),
  DONE_WITH_GROUND_INTAKE_FOR_TRANSFER(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.DoneWithGroundIntakeForTransfer.elevatorHeight)
          .armAngle(Presets.DoneWithGroundIntakeForTransfer.armAngle)
          .intakeSpeed(Presets.DoneWithGroundIntakeForTransfer.intakeSpeed)
          .groundIntakeAngle(Presets.DoneWithGroundIntakeForTransfer.groundIntakeAngle)
          .groundIntakeFrontSpeed(Presets.DoneWithGroundIntakeForTransfer.groundIntakeFrontSpeed)
          .groundIntakeBackSpeed(Presets.DoneWithGroundIntakeForTransfer.groundIntakeBackSpeed)
          .build()),
  GROUND_INTAKE_FOR_L1(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.GroundIntakeForL1.elevatorHeight)
          .armAngle(Presets.GroundIntakeForL1.armAngle)
          .intakeSpeed(Presets.GroundIntakeForL1.intakeSpeed)
          .groundIntakeAngle(Presets.GroundIntakeForL1.groundIntakeAngle)
          .groundIntakeFrontSpeed(Presets.GroundIntakeForL1.groundIntakeFrontSpeed)
          .groundIntakeBackSpeed(Presets.GroundIntakeForL1.groundIntakeBackSpeed)
          .build()),
  GROUND_INTAKE_L1(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.GroundIntakeL1.elevatorHeight)
          .armAngle(Presets.GroundIntakeL1.armAngle)
          .groundIntakeAngle(Presets.GroundIntakeL1.groundIntakeAngle)
          .groundIntakeFrontSpeed(Presets.GroundIntakeL1.groundIntakeFrontSpeed)
          .groundIntakeBackSpeed(Presets.GroundIntakeL1.groundIntakeBackSpeed)
          .build()),
  GROUND_INTAKE_L1_PRE_JERK(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.GroundIntakeL1PreJerk.elevatorHeight)
          .armAngle(Presets.GroundIntakeL1PreJerk.armAngle)
          .groundIntakeAngle(Presets.GroundIntakeL1PreJerk.groundIntakeAngle)
          .groundIntakeFrontSpeed(Presets.GroundIntakeL1PreJerk.groundIntakeFrontSpeed)
          .groundIntakeBackSpeed(Presets.GroundIntakeL1PreJerk.groundIntakeBackSpeed)
          .build()),
  GROUND_INTAKE_L1_JERK(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.GroundIntakeL1Jerk.elevatorHeight)
          .armAngle(Presets.GroundIntakeL1Jerk.armAngle)
          .groundIntakeAngle(Presets.GroundIntakeL1Jerk.groundIntakeAngle)
          .build()),

  FEEDER(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.Feeder.elevatorHeight)
          .armAngle(Presets.Feeder.armAngle)
          .intakeSpeed(Presets.Feeder.intakeSpeed)
          .groundIntakeAngle(Presets.Feeder.groundIntakeAngle)
          .groundIntakeFrontSpeed(Presets.Feeder.groundIntakeFrontSpeed)
          .groundIntakeBackSpeed(Presets.Feeder.groundIntakeBackSpeed)
          .build()),

  ARM_L1_CORAL(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.ArmL1Coral.elevatorHeight)
          .armAngle(Presets.ArmL1Coral.armAngle)
          .groundIntakeAngle(Presets.ArmL1Coral.groundIntakeAngle)
          .build()),
  ARM_L1_CORAL_OUTTAKE(
      ARM_L1_CORAL.getValue().toBuilder().intakeSpeed(Presets.IntakeSpeeds.coralOuttake).build()),
  L2_CORAL(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.L2Coral.elevatorHeight)
          .armAngle(Presets.L2Coral.armAngle)
          .groundIntakeAngle(Presets.L2Coral.groundIntakeAngle)
          .build()),
  L2_CORAL_OUTTAKE(
      L2_CORAL.getValue().toBuilder().intakeSpeed(Presets.IntakeSpeeds.coralOuttake).build()),
  L3_CORAL(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.L3Coral.elevatorHeight)
          .armAngle(Presets.L3Coral.armAngle)
          .groundIntakeAngle(Presets.L3Coral.groundIntakeAngle)
          .build()),
  L3_CORAL_OUTTAKE(
      L3_CORAL.getValue().toBuilder().intakeSpeed(Presets.IntakeSpeeds.coralOuttake).build()),

  L2_ALGAE(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.L2Algae.elevatorHeight)
          .armAngle(Presets.L2Algae.armAngle)
          .intakeSpeed(Presets.L2Algae.intakeSpeed)
          .groundIntakeAngle(Presets.L2Algae.groundIntakeAngle)
          .build()),
  L3_ALGAE(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.L3Algae.elevatorHeight)
          .armAngle(Presets.L3Algae.armAngle)
          .intakeSpeed(Presets.L3Algae.intakeSpeed)
          .groundIntakeAngle(Presets.L3Algae.groundIntakeAngle)
          .build()),

  PROCESSOR(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.Processor.elevatorHeight)
          .armAngle(Presets.Processor.armAngle)
          .intakeSpeed(Presets.Processor.intakeSpeed)
          .groundIntakeAngle(Presets.Processor.groundIntakeAngle)
          .build()),
  PROCESSOR_OUTTAKE(
      PROCESSOR.getValue().toBuilder().intakeSpeed(Presets.ProcessorOuttake.intakeSpeed).build()),
  PRE_BARGE(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.PreBarge.elevatorHeight)
          .armAngle(Presets.PreBarge.armAngle)
          .intakeSpeed(Presets.PreBarge.intakeSpeed)
          .groundIntakeAngle(Presets.PreBarge.groundIntakeAngle)
          .build()),
  BARGE(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.Barge.elevatorHeight)
          .armAngle(Presets.Barge.armAngle)
          .intakeSpeed(Presets.Barge.intakeSpeed)
          .groundIntakeAngle(Presets.Barge.groundIntakeAngle)
          .build()),

  PRE_CLIMB(
      SuperstructureStateData.builder()
          .elevatorHeight(Presets.PreClimb.elevatorHeight)
          .armAngle(Presets.PreClimb.armAngle)
          .intakeSpeed(Presets.PreClimb.intakeSpeed)
          .groundIntakeAngle(Presets.PreClimb.groundIntakeAngle)
          .groundIntakeFrontSpeed(Presets.PreClimb.groundIntakeFrontSpeed)
          .groundIntakeBackSpeed(Presets.PreClimb.groundIntakeBackSpeed)
          .build());

  private final SuperstructureStateData value;

  private SuperstructureState(SuperstructureStateData value) {
    this.value = value;
  }
}

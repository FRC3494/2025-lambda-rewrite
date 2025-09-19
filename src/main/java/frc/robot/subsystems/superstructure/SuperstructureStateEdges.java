package frc.robot.subsystems.superstructure;

import java.util.Set;

import org.jgrapht.graph.DefaultEdge;

import edu.wpi.first.wpilibj2.command.Command;
import lombok.Builder;
import lombok.Getter;

public class SuperstructureStateEdges {
  @Builder(toBuilder = true)
  @Getter
  public static class SuperstructureStateEdge extends DefaultEdge {
    private final Command command;
    @Builder.Default private final SuperstructureState restricted = null;
    @Builder.Default private final boolean holdingAlgae = false;
  }

  // Every "from" state has an edge to every "to" state
  public static final Set<SuperstructureState> safeNoAlgaeFromStates =
      Set.of(
          SuperstructureState.STOW,
          SuperstructureState.DONE_WITH_GROUND_INTAKE_FOR_TRANSFER,
          SuperstructureState.GROUND_INTAKE_L1,
          SuperstructureState.GROUND_INTAKE_L1_JERK,
          SuperstructureState.ARM_L1_CORAL,
          SuperstructureState.ARM_L1_CORAL_OUTTAKE,
          SuperstructureState.L2_CORAL,
          SuperstructureState.L2_CORAL_OUTTAKE,
          SuperstructureState.L3_CORAL,
          SuperstructureState.L3_CORAL_OUTTAKE,
          SuperstructureState.L2_ALGAE,
          SuperstructureState.L3_ALGAE,
          SuperstructureState.PROCESSOR,
          SuperstructureState.PROCESSOR_OUTTAKE,
          SuperstructureState.PRE_BARGE,
          SuperstructureState.BARGE,
          SuperstructureState.PRE_CLIMB);
  public static final Set<SuperstructureState> safeNoAlgaeToStates =
      Set.of(
          SuperstructureState.STOW,
          SuperstructureState.GROUND_INTAKE_FOR_TRANSFER,
          SuperstructureState.GROUND_INTAKE_FOR_L1,
          SuperstructureState.FEEDER,
          SuperstructureState.ARM_L1_CORAL,
          SuperstructureState.L2_CORAL,
          SuperstructureState.L3_CORAL,
          SuperstructureState.L2_ALGAE,
          SuperstructureState.L3_ALGAE,
          SuperstructureState.PRE_CLIMB);
}

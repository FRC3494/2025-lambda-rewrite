package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Set;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.graph.DefaultEdge;

public class SuperstructureStateEdges {
  @Builder(toBuilder = true)
  @Getter
  public static class SuperstructureStateEdge extends DefaultEdge {
    private final Command command;
    @Builder.Default private final SuperstructureState restricted = null;
  }

  // Every "from" state has an edge to every "to" state
  public static final Set<SuperstructureState> safeNoAlgaeFromStates =
      Set.of(
          SuperstructureState.STOW,
          SuperstructureState.GROUND_INTAKE_ARM_IN_BETWEEN,
          SuperstructureState.GROUND_INTAKE_FOR_L1,
          SuperstructureState.GROUND_INTAKE_L1,
          SuperstructureState.GROUND_INTAKE_L1_PRE_JERK,
          SuperstructureState.GROUND_INTAKE_L1_JERK,
          SuperstructureState.PRE_FEEDER,
          SuperstructureState.ARM_L1_CORAL,
          SuperstructureState.ARM_L1_CORAL_OUTTAKE,
          SuperstructureState.L2_CORAL,
          SuperstructureState.L2_CORAL_OUTTAKE,
          SuperstructureState.L3_CORAL,
          SuperstructureState.L3_CORAL_OUTTAKE,
          SuperstructureState.L2_ALGAE,
          SuperstructureState.L2_ALGAE_UP,
          SuperstructureState.L3_ALGAE,
          SuperstructureState.L3_ALGAE_UP,
          SuperstructureState.PROCESSOR,
          SuperstructureState.PROCESSOR_OUTTAKE,
          SuperstructureState.PRE_BARGE,
          SuperstructureState.BARGING,
          SuperstructureState.BARGE,
          SuperstructureState.PRE_CLIMB);
  public static final Set<SuperstructureState> safeNoAlgaeToStates =
      Set.of(
          SuperstructureState.STOW,
          SuperstructureState.GROUND_INTAKE_ARM_IN_BETWEEN,
          SuperstructureState.GROUND_INTAKE_FOR_L1,
          SuperstructureState.PRE_FEEDER,
          SuperstructureState.ARM_L1_CORAL,
          SuperstructureState.L2_CORAL,
          SuperstructureState.L3_CORAL,
          SuperstructureState.L2_ALGAE,
          SuperstructureState.L3_ALGAE,
          SuperstructureState.PRE_CLIMB);
}

package frc.robot.subsystems.superstructure;

import java.util.Set;

import org.jgrapht.graph.DefaultEdge;

import edu.wpi.first.wpilibj2.command.Command;
import lombok.Builder;
import lombok.Getter;

public class SuperstructureStateEdges {
  @Builder(toBuilder = true)
  @Getter
  public static class SuperStructureStateEdge extends DefaultEdge {
    private final Command command;
    @Builder.Default private final boolean restricted = false;
  }

  public static final Set<SuperstructureState> algaeStates = Set.of();

  public static Command getEdgeCommand(SuperstructureState from, SuperstructureState to) {}
}

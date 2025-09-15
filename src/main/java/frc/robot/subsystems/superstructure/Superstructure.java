package frc.robot.subsystems.superstructure;

import java.util.Optional;
import java.util.stream.Collectors;

import org.jgrapht.Graph;
import org.jgrapht.alg.shortestpath.BFSShortestPath;
import org.jgrapht.graph.AsSubgraph;
import org.jgrapht.graph.DefaultDirectedGraph;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

  private final Graph<SuperstructureState, EdgeCommand> graph =
      new DefaultDirectedGraph<>(EdgeCommand.class);

  private SuperstructureState currentState = SuperstructureState.IDLE;
  private SuperstructureState nextState;
  private SuperstructureState targetState = SuperstructureState.IDLE;
  private EdgeCommand currentEdge;

  public void SuperStructure() {
    for (var state : SuperstructureState.values()) {
      graph.addVertex(state);
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      targetState = currentState;
      nextState = null;
    } else if (currentEdge == null || !currentEdge.getCommand().isScheduled()) {
      if (nextState != null) {
        currentState = nextState;
        nextState = null;
      }
    }

    if (currentState != targetState) {
      getNextState(currentState, targetState)
          .ifPresent(
              (next) -> {
                if (!this.nextState.equals(next)) {
                  this.nextState = next;
                  this.currentEdge = graph.getEdge(currentState, next);
                  this.currentEdge.getCommand().schedule();
                }
              });
    }
  }

  private Optional<SuperstructureState> getNextState(
      SuperstructureState start, SuperstructureState goal) {
    Graph<SuperstructureState, EdgeCommand> graphWithoutRestrictedEdges =
        new AsSubgraph<>(
            graph,
            null,
            graph.edgeSet().stream()
                .filter(
                    (edge) -> {
                      SuperstructureState target = graph.getEdgeTarget(edge);
                      return target.equals(goal);
                    })
                .collect(Collectors.toSet()));

    BFSShortestPath<SuperstructureState, EdgeCommand> bfs =
        new BFSShortestPath<>(graphWithoutRestrictedEdges);

    SuperstructureState next = bfs.getPath(start, goal).getVertexList().get(1);
    if (next != null) {
      return Optional.of(next);
    } else {
      return Optional.empty();
    }
  }
}

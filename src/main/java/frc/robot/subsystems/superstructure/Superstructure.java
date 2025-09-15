package frc.robot.subsystems.superstructure;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

  private final Graph<SuperstructureState, EdgeCommand> graph =
      new DefaultDirectedGraph<>(EdgeCommand.class);

  private SuperstructureState currentState = SuperstructureState.IDLE;
  private SuperstructureState targetState = SuperstructureState.IDLE;

  public SuperStructure() {
    for (var state : SuperstructureState.values()) {
      graph.addVertex(state);
    }
  }
}

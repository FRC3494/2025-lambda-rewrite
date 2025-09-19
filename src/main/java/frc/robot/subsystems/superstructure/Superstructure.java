package frc.robot.subsystems.superstructure;

import java.util.Optional;

import org.jgrapht.Graph;
import org.jgrapht.alg.shortestpath.BFSShortestPath;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.MaskSubgraph;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.superstructure.SuperstructureStateEdges.SuperstructureStateEdge;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.groundintake.GroundIntake;
import frc.robot.subsystems.superstructure.intake.Intake;

public class Superstructure extends SubsystemBase {
  private final Graph<SuperstructureState, SuperstructureStateEdge> graph =
      new DefaultDirectedGraph<>(SuperstructureStateEdge.class);

  private SuperstructureState currentState = SuperstructureState.IDLE;
  private SuperstructureState nextState;
  private SuperstructureState targetState = SuperstructureState.IDLE;
  private SuperstructureStateEdge currentEdge;

  private Elevator elevator;
  private Arm arm;
  private Intake intake;
  private GroundIntake groundIntake;

  public void SuperStructure(Elevator elevator, Arm arm, Intake intake, GroundIntake groundIntake) {
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.groundIntake = groundIntake;

    for (var state : SuperstructureState.values()) {
      graph.addVertex(state);
    }

    // Graph: https://drive.google.com/file/d/1joA_YmiCZg7obwli7Sh5joll4vK8OoT4/view?usp=drive_link
    // TODO: add edges
    for (var from : SuperstructureStateEdges.safeNoAlgaeFromStates) {
      for (var to : SuperstructureStateEdges.safeNoAlgaeToStates) {
        if (!from.equals(to)) {
          graph.addEdge(
              from,
              to,
              SuperstructureStateEdge.builder()
                  .command(getEdgeCommand(from, to))
                  .restricted(null)
                  .holdingAlgae(false)
                  .build());
        }
      }
    }
  }

  @Override
  // @CodeScene(disable: "Bumpy Road Ahead")
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
    MaskSubgraph<SuperstructureState, SuperstructureStateEdge> maskedGraph =
        new MaskSubgraph<>(graph, v -> true, e -> isEdgeAllowed(e, goal));

    BFSShortestPath<SuperstructureState, SuperstructureStateEdge> bfs =
        new BFSShortestPath<>(maskedGraph);

    SuperstructureState next = bfs.getPath(start, goal).getVertexList().get(1);
    if (next != null) {
      return Optional.of(next);
    } else {
      return Optional.empty();
    }
  }

  private boolean isEdgeAllowed(SuperstructureStateEdge edge, SuperstructureState goal) {
    if (edge.getRestricted() == null) {
      return true;
    }

    if (edge.getRestricted().equals(goal)) {
      return true;
    } else {
      return false;
    }
  }

  private Command getEdgeCommand(SuperstructureState from, SuperstructureState to) {
    SuperstructureStateData toData = to.getValue();

    switch (to) {
      case STOW:
        {
          return Commands.sequence(
              intake.setSpeed(toData.getIntakeSpeed()),
              groundIntake.setIntakeSpeeds(
                  toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()),
              elevator.setTargetPosition(toData.getElevatorHeight()),
              arm.setTargetPosition(toData.getArmAngle()),
              new WaitUntilCommand(arm::pastSafePosition),
              groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()));
        }

      case GROUND_INTAKE_FOR_TRANSFER:
        {
          return Commands.sequence(
              groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()),
              intake.setSpeed(toData.getIntakeSpeed()),
              new WaitUntilCommand(groundIntake::pastSafePosition),
              arm.setTargetPosition(toData.getArmAngle()),
              elevator.setTargetPosition(toData.getElevatorHeight()),
              new WaitUntilCommand(arm::atTargetPosition),
              groundIntake.setIntakeSpeeds(
                  toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()));
        }

      default:
        {
          System.out.println("getEdgeCommand failed");
          return Commands.none();
        }
    }
  }
}

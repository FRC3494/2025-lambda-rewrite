package frc.robot.subsystems.superstructure;

import java.util.Optional;

import org.jgrapht.Graph;
import org.jgrapht.alg.shortestpath.BFSShortestPath;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.MaskSubgraph;
import org.littletonrobotics.junction.Logger;

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

  // @CodeScene(disable: "Large Method")
  public void SuperStructure(Elevator elevator, Arm arm, Intake intake, GroundIntake groundIntake) {
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.groundIntake = groundIntake;

    for (var state : SuperstructureState.values()) {
      graph.addVertex(state);
    }

    // * Graph:
    // * https://drive.google.com/file/d/1gaeQf5e-YUqPdkdRdQKs4Trm9OZLYEAZ/view?usp=drive_link
    for (var from : SuperstructureStateEdges.safeNoAlgaeFromStates) {
      for (var to : SuperstructureStateEdges.safeNoAlgaeToStates) {
        if (!from.equals(to)) {
          addEdge(from, to, null, false);
        }
      }
    }

    // ==================== Ground Intake for Transfer ====================
    addEdge(
        SuperstructureState.GROUND_INTAKE_ARM_IN_BETWEEN,
        SuperstructureState.PRE_GROUND_INTAKE_FOR_TRANSFER,
        SuperstructureState.GROUND_INTAKE_FOR_TRANSFER,
        false);
    addEdge(
        SuperstructureState.PRE_GROUND_INTAKE_FOR_TRANSFER,
        SuperstructureState.GROUND_INTAKE_FOR_TRANSFER,
        SuperstructureState.GROUND_INTAKE_FOR_TRANSFER,
        false);
    addEdge(
        SuperstructureState.GROUND_INTAKE_FOR_TRANSFER,
        SuperstructureState.DONE_WITH_GROUND_INTAKE_FOR_TRANSFER,
        null,
        false);
    addEdge(
        SuperstructureState.DONE_WITH_GROUND_INTAKE_FOR_TRANSFER,
        SuperstructureState.GROUND_INTAKE_ARM_IN_BETWEEN,
        null,
        false);

    // ==================== Ground Intake for L1 ====================
    addEdge(
        SuperstructureState.GROUND_INTAKE_FOR_L1,
        SuperstructureState.GROUND_INTAKE_L1,
        SuperstructureState.GROUND_INTAKE_L1,
        false);
    addEdge(
        SuperstructureState.GROUND_INTAKE_L1,
        SuperstructureState.GROUND_INTAKE_L1_PRE_JERK,
        SuperstructureState.GROUND_INTAKE_L1_JERK,
        false);
    addEdge(
        SuperstructureState.GROUND_INTAKE_L1_PRE_JERK,
        SuperstructureState.GROUND_INTAKE_L1_JERK,
        SuperstructureState.GROUND_INTAKE_L1_JERK,
        false);

    // ==================== Feeder ====================
    addEdge(
        SuperstructureState.PRE_FEEDER,
        SuperstructureState.FEEDER,
        SuperstructureState.FEEDER,
        false);
    addEdge(SuperstructureState.FEEDER, SuperstructureState.PRE_FEEDER, null, false);

    // ==================== Coral Outtake w/ Arm ====================
    addEdge(
        SuperstructureState.ARM_L1_CORAL,
        SuperstructureState.ARM_L1_CORAL_OUTTAKE,
        SuperstructureState.ARM_L1_CORAL_OUTTAKE,
        false);
    addEdge(
        SuperstructureState.L2_CORAL,
        SuperstructureState.L2_CORAL_OUTTAKE,
        SuperstructureState.L2_CORAL_OUTTAKE,
        false);
    addEdge(
        SuperstructureState.L3_CORAL,
        SuperstructureState.L3_CORAL_OUTTAKE,
        SuperstructureState.L3_CORAL_OUTTAKE,
        false);

    // ==================== Algae Intake ====================
    // * `holdingAlgae` is false because we don't want passive algae intake when moving between
    // * normal and higher versions of algae intake
    addEdge(
        SuperstructureState.L2_ALGAE,
        SuperstructureState.L2_ALGAE_UP,
        SuperstructureState.L2_ALGAE_UP,
        false);
    addEdge(
        SuperstructureState.L2_ALGAE_UP,
        SuperstructureState.L2_ALGAE,
        SuperstructureState.L2_ALGAE,
        false);

    addEdge(
        SuperstructureState.L3_ALGAE,
        SuperstructureState.L3_ALGAE_UP,
        SuperstructureState.L3_ALGAE_UP,
        false);
    addEdge(
        SuperstructureState.L3_ALGAE_UP,
        SuperstructureState.L3_ALGAE,
        SuperstructureState.L3_ALGAE,
        false);

    // ==================== Processor ====================
    addEdge(
        SuperstructureState.L2_ALGAE,
        SuperstructureState.PROCESSOR,
        SuperstructureState.PROCESSOR,
        true);
    addEdge(
        SuperstructureState.L2_ALGAE_UP,
        SuperstructureState.PROCESSOR,
        SuperstructureState.PROCESSOR,
        true);
    addEdge(
        SuperstructureState.L3_ALGAE,
        SuperstructureState.PROCESSOR,
        SuperstructureState.PROCESSOR,
        true);
    addEdge(
        SuperstructureState.L3_ALGAE_UP,
        SuperstructureState.PROCESSOR,
        SuperstructureState.PROCESSOR,
        true);

    // * `holdingAlgae` is false because we don't want passive algae intake while outtaking
    addEdge(
        SuperstructureState.PROCESSOR,
        SuperstructureState.PROCESSOR_OUTTAKE,
        SuperstructureState.PROCESSOR_OUTTAKE,
        false);

    // ==================== Barge ====================
    addEdge(
        SuperstructureState.L2_ALGAE,
        SuperstructureState.PRE_BARGE,
        SuperstructureState.PRE_BARGE,
        true);
    addEdge(
        SuperstructureState.L2_ALGAE_UP,
        SuperstructureState.PRE_BARGE,
        SuperstructureState.PRE_BARGE,
        true);
    addEdge(
        SuperstructureState.L3_ALGAE,
        SuperstructureState.PRE_BARGE,
        SuperstructureState.PRE_BARGE,
        true);
    addEdge(
        SuperstructureState.L3_ALGAE_UP,
        SuperstructureState.PRE_BARGE,
        SuperstructureState.PRE_BARGE,
        true);

    // * `holdingAlgae` is false because we don't want passive algae intake while outtaking
    addEdge(
        SuperstructureState.PRE_BARGE,
        SuperstructureState.BARGING,
        SuperstructureState.BARGE,
        false);
    addEdge(
        SuperstructureState.BARGING, SuperstructureState.BARGE, SuperstructureState.BARGE, false);
  }

  @Override
  // @CodeScene(disable: "Bumpy Road Ahead")
  public void periodic() {
    if (DriverStation.isDisabled()) {
      targetState = currentState;
      nextState = null;
    } else if (currentEdge == null || !currentEdge.getCommand().isScheduled()) {
      if (nextState != null) {
        currentEdge = graph.getEdge(currentState, nextState);
        currentEdge.getCommand().schedule();

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
                }
              });
    }

    Logger.recordOutput("Superstructure/CurrentState", currentState.toString());
    Logger.recordOutput("Superstructure/NextState", nextState.toString());
    Logger.recordOutput("Superstructure/TargetState", targetState.toString());
  }

  private Optional<SuperstructureState> getNextState(
      SuperstructureState start, SuperstructureState goal) {
    MaskSubgraph<SuperstructureState, SuperstructureStateEdge> maskedGraph =
        new MaskSubgraph<>(graph, v -> true, e -> isEdgeAllowedUnderRestrictions(e, goal));

    BFSShortestPath<SuperstructureState, SuperstructureStateEdge> bfs =
        new BFSShortestPath<>(maskedGraph);

    SuperstructureState next = bfs.getPath(start, goal).getVertexList().get(1);
    if (next != null) {
      return Optional.of(next);
    } else {
      return Optional.empty();
    }
  }

  private boolean isEdgeAllowedUnderRestrictions(
      SuperstructureStateEdge edge, SuperstructureState goal) {
    if (edge.getRestricted() == null) {
      return true;
    }

    if (edge.getRestricted().equals(goal)) {
      return true;
    } else {
      return false;
    }
  }

  private void addEdge(
      SuperstructureState from,
      SuperstructureState to,
      SuperstructureState restricted,
      boolean holdingAlgae) {
    graph.addEdge(
        from,
        to,
        SuperstructureStateEdge.builder()
            .command(getEdgeCommand(from, to, holdingAlgae))
            .restricted(restricted)
            .build());
  }

  private Command getEdgeCommand(
      SuperstructureState from, SuperstructureState to, boolean holdingAlgae) {
    SuperstructureStateData toData = to.getValue();

    Command goToState =
        Commands.sequence(
            elevator.setTargetPosition(toData.getElevatorHeight()),
            arm.setTargetPosition(toData.getArmAngle()),
            groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()),
            groundIntake.setIntakeSpeeds(
                toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()),
            new WaitUntilCommand(this::atSetpoint));

    if (holdingAlgae) {
      return Commands.sequence(
          intake.setSpeed(SuperstructureState.IntakeSpeeds.algaePassiveIntakeFast),
          goToState,
          intake.setSpeed(toData.getIntakeSpeed()));
    } else {
      return Commands.sequence(intake.setSpeed(toData.getIntakeSpeed()), goToState);
    }
  }

  public boolean atSetpoint() {
    return elevator.atSetpoint() && arm.atSetpoint() && groundIntake.atSetpoint();
  }
}

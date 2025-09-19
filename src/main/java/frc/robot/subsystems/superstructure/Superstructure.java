package frc.robot.subsystems.superstructure;

import java.util.Optional;
import java.util.function.BooleanSupplier;

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
          if (SuperstructureStateEdges.safeNoAlgaeFromStates.contains(from)) {
            return Commands.sequence(
                intake.setSpeed(toData.getIntakeSpeed()),
                groundIntake.setIntakeSpeeds(
                    toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()),
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                new WaitUntilCommand((BooleanSupplier) arm::pastSafePosition),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()));
          } else if (from == SuperstructureState.FEEDER) {
            return Commands.sequence(
                intake.setSpeed(toData.getIntakeSpeed()),
                groundIntake.setIntakeSpeeds(
                    toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()),
                // TODO: do smth else with where this constant comes from?
                arm.setTargetPosition(Presets.Feeder.armSafeAngle),
                elevator.setTargetPosition(toData.getElevatorHeight()),
                new WaitUntilCommand((BooleanSupplier) arm::pastSafePosition),
                arm.setTargetPosition(toData.getArmAngle()),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      // ==================== Ground Intake ====================
      case GROUND_INTAKE_FOR_TRANSFER:
        {
          if (SuperstructureStateEdges.safeNoAlgaeFromStates.contains(from)) {
            return Commands.sequence(
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()),
                intake.setSpeed(toData.getIntakeSpeed()),
                // TODO: test without this line
                new WaitUntilCommand((BooleanSupplier) groundIntake::pastSafePosition),
                arm.setTargetPosition(toData.getArmAngle()),
                elevator.setTargetPosition(toData.getElevatorHeight()),
                new WaitUntilCommand((BooleanSupplier) arm::atTargetPosition),
                groundIntake.setIntakeSpeeds(
                    toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }
      case DONE_WITH_GROUND_INTAKE_FOR_TRANSFER:
        {
          if (from == SuperstructureState.GROUND_INTAKE_FOR_TRANSFER) {
            return Commands.sequence(
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()),
                groundIntake.setIntakeSpeeds(
                    toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()),
                intake.setSpeed(toData.getIntakeSpeed()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      case GROUND_INTAKE_FOR_L1:
        {
          if (SuperstructureStateEdges.safeNoAlgaeFromStates.contains(from)) {
            return Commands.sequence(
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()),
                arm.setTargetPosition(toData.getArmAngle()),
                elevator.setTargetPosition(toData.getElevatorHeight()),
                intake.setSpeed(toData.getIntakeSpeed()),
                groundIntake.setIntakeSpeeds(
                    toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }
      case GROUND_INTAKE_L1:
        {
          if (from == SuperstructureState.GROUND_INTAKE_FOR_L1) {
            return Commands.sequence(
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()),
                groundIntake.setIntakeSpeeds(
                    toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }
      case GROUND_INTAKE_L1_PRE_JERK:
        {
          if (from == SuperstructureState.GROUND_INTAKE_L1) {
            return Commands.sequence(
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()),
                groundIntake.setIntakeSpeeds(
                    toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }
      case GROUND_INTAKE_L1_JERK:
        {
          if (from == SuperstructureState.GROUND_INTAKE_L1_PRE_JERK) {
            return Commands.sequence(
                // TODO: make speed faster? idk
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle())
                // TODO: set intake speeds? idk if we need it
                );
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      // ==================== Coral Intake w/ Arm ====================
      case FEEDER:
        {
          if (SuperstructureStateEdges.safeNoAlgaeFromStates.contains(from)) {
            return Commands.sequence(
                groundIntake.setIntakeSpeeds(
                    toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()),
                // TODO: do smth else with where this constant comes from?
                arm.setTargetPosition(Presets.Feeder.armSafeAngle),
                elevator.setTargetPosition(toData.getElevatorHeight()),
                new WaitUntilCommand((BooleanSupplier) arm::pastSafePosition),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()),
                arm.setTargetPosition(toData.getArmAngle()),
                intake.setSpeed(toData.getIntakeSpeed()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      // ==================== Coral Outtake w/ Arm ====================
      case ARM_L1_CORAL:
        {
          if (SuperstructureStateEdges.safeNoAlgaeFromStates.contains(from)) {
            return Commands.sequence(
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                new WaitUntilCommand((BooleanSupplier) arm::pastSafePosition),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }
      case ARM_L1_CORAL_OUTTAKE:
        {
          if (from == SuperstructureState.ARM_L1_CORAL) {
            return intake.setSpeed(toData.getIntakeSpeed());
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      case L2_CORAL:
        {
          if (SuperstructureStateEdges.safeNoAlgaeFromStates.contains(from)) {
            return Commands.sequence(
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                new WaitUntilCommand((BooleanSupplier) arm::pastSafePosition),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }
      case L2_CORAL_OUTTAKE:
        {
          if (from == SuperstructureState.L2_CORAL) {
            return intake.setSpeed(toData.getIntakeSpeed());
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      case L3_CORAL:
        {
          if (SuperstructureStateEdges.safeNoAlgaeFromStates.contains(from)) {
            return Commands.sequence(
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                new WaitUntilCommand((BooleanSupplier) arm::pastSafePosition),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }
      case L3_CORAL_OUTTAKE:
        {
          if (from == SuperstructureState.L3_CORAL) {
            return intake.setSpeed(toData.getIntakeSpeed());
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      // ==================== Algae Intake ====================
      case L2_ALGAE:
        {
          if (SuperstructureStateEdges.safeNoAlgaeFromStates.contains(from)) {
            return Commands.sequence(
                intake.setSpeed(toData.getIntakeSpeed()),
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                new WaitUntilCommand((BooleanSupplier) arm::pastSafePosition),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      case L3_ALGAE:
        {
          if (SuperstructureStateEdges.safeNoAlgaeFromStates.contains(from)) {
            return Commands.sequence(
                intake.setSpeed(toData.getIntakeSpeed()),
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                new WaitUntilCommand((BooleanSupplier) arm::pastSafePosition),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      // ==================== Algae Outtake ====================
      case PROCESSOR:
        {
          if (from == SuperstructureState.L2_ALGAE || from == SuperstructureState.L3_ALGAE) {
            return Commands.sequence(
                intake.setSpeed(toData.getIntakeSpeed()),
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                new WaitUntilCommand((BooleanSupplier) arm::pastSafePosition),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }
      case PROCESSOR_OUTTAKE:
        {
          if (from == SuperstructureState.PROCESSOR) {
            return intake.setSpeed(toData.getIntakeSpeed());
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      case PRE_BARGE:
        {
          if (from == SuperstructureState.L2_ALGAE || from == SuperstructureState.L3_ALGAE) {
            return Commands.sequence(
                intake.setSpeed(Presets.PreBarge.intakeSpeedDuring),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()),
                arm.setTargetPosition(toData.getArmAngle()),
                elevator.setTargetPosition(toData.getElevatorHeight()),
                new WaitUntilCommand(arm::atTargetPosition),
                intake.setSpeed(toData.getIntakeSpeed()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }
      case BARGE:
        {
          if (from == SuperstructureState.PRE_BARGE) {
            return Commands.sequence(
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle()),
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                // TODO: check if less than or greater than
                new WaitUntilCommand(() -> arm.getPosition() < Presets.Barge.bargeYeetReleaseAngle),
                intake.setSpeed(toData.getIntakeSpeed()));
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      // ==================== Pre-Climb ====================
      case PRE_CLIMB:
        {
          if (SuperstructureStateEdges.safeNoAlgaeFromStates.contains(from)) {
            return Commands.sequence(
                intake.setSpeed(toData.getIntakeSpeed()),
                groundIntake.setIntakeSpeeds(
                    toData.getGroundIntakeFrontSpeed(), toData.getGroundIntakeBackSpeed()),
                elevator.setTargetPosition(toData.getElevatorHeight()),
                arm.setTargetPosition(toData.getArmAngle()),
                new WaitUntilCommand((BooleanSupplier) arm::pastSafePosition),
                groundIntake.setTargetPivotPosition(toData.getGroundIntakeAngle())
                // TODO: climber
                );
          } else {
            printNoEdgeMessage(from, to);
            return Commands.none();
          }
        }

      default:
        {
          System.out.println("getEdgeCommand: No edges exist from state: " + from.toString());
          return Commands.none();
        }
    }
  }

  private void printNoEdgeMessage(SuperstructureState from, SuperstructureState to) {
    System.out.println(
        "No edge exists from state: " + from.toString() + " to state: " + to.toString());
  }
}

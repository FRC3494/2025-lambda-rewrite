package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.graph.DefaultEdge;

@Builder(toBuilder = true)
@Getter
public class EdgeCommand extends DefaultEdge {
  private final Command command;
  @Builder.Default private final boolean restricted = false;
}

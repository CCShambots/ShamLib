package frc.robot.ShamLib.testing;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShamLib.SMF.graph.DirectionalEnumGraph;
import frc.robot.ShamLib.SMF.transitions.CommandTransition;
import org.junit.Test;
import org.junit.jupiter.api.BeforeAll;

public class DirectionalEnumGraphTesting {
    DirectionalEnumGraph<ExampleState, CommandTransition<ExampleState>> graph;

    @BeforeAll
    public void setUp() {
        //gross
        graph = new DirectionalEnumGraph<>(new CommandTransition(ExampleState.A, ExampleState.A, new InstantCommand()).getClass(), ExampleState.class);
    }

    @Test
    public void addAndGetEdge() {

    }
}

enum ExampleState {
    A,
    B,
    C,
    D,
    E
}
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
        CommandTransition[] transitions = {
                new CommandTransition<>(ExampleState.A, ExampleState.B, new InstantCommand()),
                new CommandTransition<>(ExampleState.D, ExampleState.B, new InstantCommand()),
                new CommandTransition<>(ExampleState.D, ExampleState.C, new InstantCommand()),
                new CommandTransition<>(ExampleState.C, ExampleState.D, new InstantCommand())
        };

        for (CommandTransition<ExampleState> t : transitions) {
            graph.addEdge(t);
            assert graph.getEdge(t.getStartState(), t.getEndState()) == t;
        }

        assert graph.getEdge(ExampleState.C, ExampleState.E) == null;
    }

    @Test
    public void getEdges() {
        CommandTransition[] transitions = {
                new CommandTransition<>(ExampleState.A, ExampleState.B, new InstantCommand()),
                new CommandTransition<>(ExampleState.D, ExampleState.B, new InstantCommand()),
                new CommandTransition<>(ExampleState.D, ExampleState.C, new InstantCommand()),
                new CommandTransition<>(ExampleState.C, ExampleState.D, new InstantCommand())
        };

        for (CommandTransition<ExampleState> t : transitions) {
            graph.addEdge(t);
        }


    }
}

enum ExampleState {
    A, B, C, D, E
}
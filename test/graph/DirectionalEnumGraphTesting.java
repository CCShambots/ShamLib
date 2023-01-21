package frc.robot.ShamLib.test.graph;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShamLib.SMF.graph.DirectionalEnumGraph;
import frc.robot.ShamLib.SMF.graph.EdgeType;
import frc.robot.ShamLib.SMF.transitions.CommandTransition;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;

public class DirectionalEnumGraphTesting {
    private static DirectionalEnumGraph<ExampleState, CommandTransition<ExampleState>> graph;

    @BeforeAll
    public static void setUp() {
        //gross
        graph = new DirectionalEnumGraph<>(ExampleState.class);
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

        graph.addEdges(transitions);

        assert graph.getEdges(ExampleState.A, EdgeType.All).size() == 1;
        assert graph.getEdges(ExampleState.A, EdgeType.Incoming).size() == 0;

        assert graph.getEdges(ExampleState.D, EdgeType.All).size() == 3;
        assert graph.getEdges(ExampleState.D, EdgeType.Outgoing).size() == 2;
        assert graph.getEdges(ExampleState.D, EdgeType.Incoming).size() == 1;

        assert graph.getEdges(ExampleState.E, EdgeType.All).size() == 0;
    }

    @Test
    public void setEdge() {
        CommandTransition<ExampleState> t1 = new CommandTransition<>(ExampleState.A, ExampleState.B, new InstantCommand());
        CommandTransition<ExampleState> t2 = new CommandTransition<>(ExampleState.A, ExampleState.B, new InstantCommand());

        graph.addEdge(t1);
        graph.setEdge(t2);

        assert graph.getEdge(t2.getStartState(), t2.getEndState()) == t2;
    }
}

enum ExampleState {
    A, B, C, D, E
}
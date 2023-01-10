package frc.robot.ShamLib.SMF.tests;

import frc.robot.ShamLib.SMF.graph.DirectionalEnumGraph;
import frc.robot.ShamLib.SMF.graph.EdgeType;
import frc.robot.ShamLib.SMF.states.StateBase;
import frc.robot.ShamLib.SMF.transitions.InstantTransition;
import frc.robot.ShamLib.SMF.transitions.TransitionBase;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.function.BooleanSupplier;

class DirectionalEnumGraphTest {
    DirectionalEnumGraph<ExampleState, DummyTransition> graph;

    @BeforeAll
    void setUp() {
        graph = new DirectionalEnumGraph<>(DummyTransition.class, ExampleState.class);
    }


    @Test
    void addAndGetEdge() {
        DummyTransition d0 = new DummyTransition(ExampleState.A, ExampleState.B);
        DummyTransition d1 = new DummyTransition(ExampleState.B, ExampleState.A);

        graph.addEdge(d0);
        graph.addEdge(d1);

        assert graph.getEdge(d0.getStartState(), d0.getEndState()) == d0;
        assert graph.getEdge(d1.getStartState(), d1.getEndState()) == d1;
        assert graph.getEdge(ExampleState.A, ExampleState.C) == null;
        assert graph.getEdge(ExampleState.A, ExampleState.A) == null;

    }

    @Test
    void setEdge() {
        DummyTransition d0 = new DummyTransition(ExampleState.A, ExampleState.B);
        DummyTransition d1 = new DummyTransition(ExampleState.B, ExampleState.A);
        DummyTransition d2 = new DummyTransition(ExampleState.B, ExampleState.A);

        graph.addEdge(d0);
        graph.addEdge(d1);

        graph.setEdge(d2);

        assert graph.getEdge(d2.getStartState(), d2.getEndState()) == d2;

        graph.setEdge(d1);

        assert graph.getEdge(d1.getStartState(), d1.getEndState()) == d1;
    }

    @Test
    void getEdges() {
        DummyTransition[] t = {
                new DummyTransition(ExampleState.A, ExampleState.B),
                new DummyTransition(ExampleState.B, ExampleState.A),
                new DummyTransition(ExampleState.B, ExampleState.C),
                new DummyTransition(ExampleState.C, ExampleState.A)
        };

        for (DummyTransition d : t) {
            graph.addEdge(d);
        }

        List<DummyTransition> outFromB = graph.getEdges(ExampleState.B, EdgeType.Outgoing);
        List<DummyTransition> inToA = graph.getEdges(ExampleState.A, EdgeType.Incoming);
        int check1 = 0b0110;
        int check2 = 0b1010;
    }

    private class DummyTransition extends TransitionBase<ExampleState> {

        public DummyTransition(ExampleState startState, ExampleState endState) {
            super(startState, endState);
        }

        @Override
        public String toString() {
            return null;
        }

        @Override
        public BooleanSupplier execute() {
            return null;
        }

        @Override
        public void cancel() {

        }
    }

    enum ExampleState {
        A,
        B,
        C
    }
}
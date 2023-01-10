package frc.robot.ShamLib.SMF.tests;

import frc.robot.ShamLib.SMF.graph.DirectionalGraph;
import frc.robot.ShamLib.SMF.transitions.InstantTransition;
import frc.robot.ShamLib.SMF.transitions.TransitionBase;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class DirectionalGraphTest {
    DirectionalGraph<ExampleState, InstantTransition<ExampleState>> graph;

    @BeforeAll
    void setUp() {
        graph = new DirectionalGraph<>(InstantTransition.class);
    }

    @Test
    void addVertex() {
    }

    @Test
    void addDirectionalEdge() {
    }

    @Test
    void setDirectionalEdge() {
    }

    @Test
    void getDirectionalEdge() {
    }

    @Test
    void getVertices() {
    }

    @Test
    void getEdges() {
    }

    enum ExampleState {
        A,
        B,
        C
    }
}
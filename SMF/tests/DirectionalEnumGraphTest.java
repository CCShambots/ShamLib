package frc.robot.ShamLib.SMF.tests;

import frc.robot.ShamLib.SMF.graph.DirectionalEnumGraph;
import frc.robot.ShamLib.SMF.transitions.InstantTransition;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DirectionalEnumGraphTest {
    DirectionalEnumGraph<ExampleState, InstantTransition<ExampleState>> graph;

    @BeforeAll
    void setUp() {
        //graph = new DirectionalEnumGraph<>(InstantTransition.class);
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
package frc.robot.ShamLib.SMF.graph;

import frc.robot.ShamLib.SMF.graph.exceptions.ExistingEdgeException;
import frc.robot.ShamLib.SMF.graph.exceptions.UnfoundVertexException;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class DirectionalGraph<V, E, T extends Enum<T>> {
    private Map<T, Vertex<V, E, T>> vertices = new HashMap<>();

    private Supplier<V> defaultVertexValue;

    public DirectionalGraph(Supplier<V> defaultVertexValue) {
        this.defaultVertexValue = defaultVertexValue;
    }

    public Vertex<V, E, T> addVertex(T enumValue, V value) {
        Vertex<V, E, T> v = new Vertex<>(enumValue, value);
        vertices.put(enumValue, v);
        return v;
    }

    public Vertex<V, E, T> findVertex(T enumValue) {
        return vertices.get(enumValue);
    }

    public void addEdge(T start, T end, E edgeValue) throws ExistingEdgeException {
        Vertex<V, E, T> startVertex = vertices.get(start);
        Vertex<V, E, T> endVertex = vertices.get(end);

        if(startVertex == null) addVertex(start, defaultVertexValue.get());
        if(endVertex == null) addVertex(end, defaultVertexValue.get());

        startVertex.addEdge(new DirectionalEdge<>(startVertex, endVertex, edgeValue));
    }

    public DirectionalEdge<V, E, T> findEdge(T start, T end) throws UnfoundVertexException {
        Vertex<V, E, T> startVertex = vertices.get(start);
        Vertex<V, E, T> endVertex = vertices.get(end);

        if(startVertex == null || endVertex == null) {
            return startVertex.findEdge(endVertex);
        } else throw new UnfoundVertexException();
    }
}

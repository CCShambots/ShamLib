package frc.robot.ShamLib.SMF.graph;

import frc.robot.ShamLib.SMF.graph.exceptions.ExistingEdgeException;
import frc.robot.ShamLib.SMF.graph.exceptions.UnfoundVertexException;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

public class DirectionalGraph<V, E, T extends Enum<T>> {
    private Map<T, Vertex<V, E, T>> vertices = new HashMap<>();

    private Function<T, V> defaultVertexFunction;

    public DirectionalGraph(Function<T, V> defaultVertexFunction) {
        this.defaultVertexFunction = defaultVertexFunction;

    }

    private <T, V> Supplier<V> bind(Function<T, V> fn, T val) {
        return () -> fn.apply(val);
    }

    public Vertex<V, E, T> addVertex(T enumValue, V value) {
        Vertex<V, E, T> v = new Vertex<>(enumValue, value);
        vertices.put(enumValue, v);
        return v;
    }

    private Vertex<V, E, T> createDefaultVertex(T enumValue) {
        return addVertex(enumValue, bind(defaultVertexFunction, enumValue).get());
    }

    public Vertex<V, E, T> findVertex(T enumValue) {
        return vertices.get(enumValue);
    }

    public Vertex<V, E, T> findOrCreateVertex(T enumValue) {
        Vertex<V, E, T> fromlist = findVertex(enumValue);

        if(fromlist != null) return fromlist;

        return createDefaultVertex(enumValue);
    }

    public void addEdge(T start, T end, E edgeValue) throws ExistingEdgeException {
        Vertex<V, E, T> startVertex = vertices.get(start);
        Vertex<V, E, T> endVertex = vertices.get(end);

        if(startVertex == null) createDefaultVertex(start);
        if(endVertex == null) createDefaultVertex(end);

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

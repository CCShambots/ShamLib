package frc.robot.ShamLib.SMF.graph;

import frc.robot.ShamLib.SMF.graph.exceptions.ExistingEdgeException;
import frc.robot.ShamLib.SMF.graph.exceptions.UnfoundVertexException;

import java.util.*;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class DirectionalGraph<V, E, T extends Enum<T>> {
    private final Map<T, Vertex<V, E, T>> vertices = new HashMap<>();
    private final List<DirectionalEdge<V, E, T>> edges = new ArrayList<>(); // Just for storage

    private final Function<T, V> defaultVertexFunction;

    private final Runnable toRunOnChange;

    public DirectionalGraph(Function<T, V> defaultVertexFunction, Runnable toRunOnChange) {
        this.defaultVertexFunction = defaultVertexFunction;
        this.toRunOnChange = toRunOnChange;
    }

    public DirectionalGraph(Function<T, V> defaultVertexFunction) {
        this(defaultVertexFunction, () -> {});
    }

    private Supplier<V> bind(Function<T, V> fn, T val) {
        return () -> fn.apply(val);
    }

    public Vertex<V, E, T> addVertex(T enumValue, V value) {
        Vertex<V, E, T> v = new Vertex<>(enumValue, value);
        vertices.put(enumValue, v);

        toRunOnChange.run();
        return v;
    }

    private Vertex<V, E, T> createDefaultVertex(T enumValue) {
        return addVertex(enumValue, bind(defaultVertexFunction, enumValue).get());
    }

    public Vertex<V, E, T> findVertex(T enumValue) {
        return vertices.get(enumValue);
    }

    public Vertex<V, E, T> findOrCreateVertex(T enumValue) {
        Vertex<V, E, T> fromList = findVertex(enumValue);

        if(fromList != null) return fromList;

        return createDefaultVertex(enumValue);
    }

    public void addEdge(T start, T end, E edgeValue) throws ExistingEdgeException {
        Vertex<V, E, T> startVertex = vertices.get(start);
        Vertex<V, E, T> endVertex = vertices.get(end);

        if(startVertex == null) createDefaultVertex(start);
        if(endVertex == null) createDefaultVertex(end);
        DirectionalEdge<V, E, T> edge = new DirectionalEdge<>(startVertex, endVertex, edgeValue);

        startVertex.addEdge(edge);
        edges.add(edge);

        toRunOnChange.run();
    }

    public DirectionalEdge<V, E, T> findEdge(T start, T end) throws UnfoundVertexException {
        Vertex<V, E, T> startVertex = vertices.get(start);
        Vertex<V, E, T> endVertex = vertices.get(end);

        if(startVertex == null || endVertex == null) {
            return startVertex.findEdge(endVertex);
        } else throw new UnfoundVertexException();
    }

    public Map<T, Vertex<V, E, T>> getVertices() {
        return vertices;
    }

    public List<DirectionalEdge<V, E, T>> getEdges() {
        return edges;
    }

    public Set<Vertex<V, E, T>> getVerticesWithCondition(Predicate<Vertex<V, E, T>> fn) {
       return vertices.values().stream().filter(fn).collect(Collectors.toSet());
    }

    public Set<V> getVertexObjectsWithCondition(Predicate<V> condition) {
        return vertices.values().stream().map(Vertex::getValue).filter(condition).collect(Collectors.toSet());
    }

}
package frc.robot.ShamLib.SMF.graph;

import frc.robot.ShamLib.SMF.graph.exceptions.ExistingEdgeException;

import java.util.HashMap;
import java.util.Map;

public class Vertex<V, E, T extends Enum<T>> {
    //Map to pair end states with edges
    private Map<Vertex<V,E,T>, DirectionalEdge<V, E, T>> edges = new HashMap<>();

    private T enumValue;
    private V value;

    public Vertex(T enumValue, V value) {
        this.enumValue = enumValue;
        this.value = value;
    }

    void addEdge(DirectionalEdge<V, E, T> edge) throws ExistingEdgeException {
        for(DirectionalEdge<V, E, T> e : edges.values()) {
            if(edge.equals(e)) throw new ExistingEdgeException();
        }

        edges.put(edge.getEnd(), edge);
    }

    DirectionalEdge<V, E, T> findEdge(Vertex<V, E, T> end) {
        return edges.get(end);
    }

    public T getEnumValue() {
        return enumValue;
    }

    public V getValue() {
        return value;
    }

    public void setValue(V value) {
        this.value = value;
    }
}

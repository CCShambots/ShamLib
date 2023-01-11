package frc.robot.ShamLib.SMF.graph;

import frc.robot.ShamLib.SMF.transitions.TransitionBase;

import java.lang.reflect.Array;
import java.util.*;

public class DirectionalEnumGraph<V extends Enum<V>, T extends TransitionBase<? extends Enum<V>>> {
    //Array where the index [1][2] = TransitionBase from state at ordinal 1 to state at ordinal 2
    private final T[][] adjacencyMap;
    private final Class<V> enumType;

    public DirectionalEnumGraph(Class<T> transitionType, Class<V> enumType) {
        int c = enumType.getEnumConstants().length;
        this.enumType = enumType;
        
        //TODO: optimize this somehow because it feels messy?
        //My thought is that it's probably fine. You're write that it's a little gross, but I think it's efficient and good - Barta
        adjacencyMap = (T[][]) Array.newInstance(transitionType, c, c);
    }

    /**
     * Update the adjacency map to include the data for the given transition
     * Note: This WILL NOT overwrite an existing edge
     * @param transition The transition to add to the graph
     */
    public void addEdge(T transition) {
        //TODO: Should we be throwing an exception here or at least returning a boolean of success or not?
        if (getEdge(fromOrdinal(transition.getStartValue().ordinal()), fromOrdinal(transition.getEndValue().ordinal())) != null) return;

        setEdge(transition);
    }

    private V fromOrdinal(int ordinal) {
        return enumType.getEnumConstants()[ordinal];
    }

    /**
     * Update the adjacency map to include the data for the given transition.
     * Note: This WILL override an existing transition, if one is present
     * @param transition The transition to set to the graph
     */
    public void setEdge(T transition) {
        adjacencyMap[transition.getStartValue().ordinal()][transition.getEndValue().ordinal()] = transition;
    }

    /**
     * Returns the edge of the graph
     * @param start starting vertex of the edge
     * @param end ending vertex of the edge
     * @return the edge, if one is found. Otherwise, the method will return null
     */
    public T getEdge(V start, V end) {
        return adjacencyMap[start.ordinal()][end.ordinal()];
    }

    /**
     * Returns the edges of the graph either going to, from, or both to and from a vertex of the graph
     * @param vertex the vertex to search
     * @param t the type of resultant edges to return
     * @return the edges of the graph that match the conditions
     */
    public List<T> getEdges(V vertex, EdgeType t) {
        List<T> outgoing = new ArrayList<>();
        List<T> incoming = new ArrayList<>();

        for (int i = 0; i < adjacencyMap.length; i++) {
            T out = adjacencyMap[i][vertex.ordinal()];
            T in = adjacencyMap[vertex.ordinal()][i];
            if (out != null) outgoing.add(out);
            if (in != null) incoming.add(in);
        }

        switch (t) {
            case Incoming:
                return incoming;
            case Outgoing:
                return outgoing;
            default: //Return both
                outgoing.addAll(incoming);
                return outgoing;
        }
    }
}
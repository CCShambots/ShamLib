package frc.robot.ShamLib.SMF.graph;

import frc.robot.ShamLib.SMF.transitions.TransitionBase;

import java.lang.reflect.Array;
import java.util.*;

public class DirectionalEnumGraph<V extends Enum<V>, T extends TransitionBase<V>> {
    //TODO: rename transition to edge and state to node
    private T[][] adjacencyMap;
    List<T> edges;

    public DirectionalEnumGraph(Class<T> transitionType, Class<V> enumType) {
        int c = enumType.getEnumConstants().length;

        //this is kinda gross but idk what else to do
        adjacencyMap = (T[][]) Array.newInstance(transitionType, c, c);
        edges = new ArrayList<>();
    }

    public void addEdge(T transition) {
        if (getEdge(transition.getStartValue(), transition.getEndValue()) != null) return;

        setEdge(transition);
    }

    public void setEdge(T transition) {
        adjacencyMap[transition.getStartValue().ordinal()][transition.getEndValue().ordinal()] = transition;
    }

    public T getEdge(V start, V end) {
        return adjacencyMap[start.ordinal()][end.ordinal()];
    }

    public List<T> getAllEdges() {
        return edges;
    }
    public List<T> getEdges(V state, EdgeType t) {
        ArrayList<T> outgoing = new ArrayList<>();
        ArrayList<T> incoming = new ArrayList<>();

        for (int i = 0; i < adjacencyMap.length; i++) {
            T out = adjacencyMap[i][state.ordinal()];
            T in = adjacencyMap[state.ordinal()][i];
            if (out != null) outgoing.add(out);
            if (in != null) incoming.add(in);
        }

        switch (t) {
            case Incoming:
                return incoming;
            case Outgoing:
                return outgoing;
            default:
                outgoing.addAll(incoming);
                return outgoing;
        }
    }
}
package frc.robot.ShamLib.SMF.graph;

import frc.robot.ShamLib.SMF.transitions.TransitionBase;

import java.lang.reflect.Array;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class DirectionalGraph<V extends Enum<V>, T extends TransitionBase<V>> {
    //TODO: rename transition to edge and state to node
    private List<List<T>> adjacencyMap;
    private HashMap<V, Integer> valIndexMap;
    private List<T> transitions;
    private Class<?> transitionType;

    public DirectionalGraph(Class<?> type) {
        adjacencyMap = new ArrayList<>();
        valIndexMap = new HashMap<>();

        transitionType = type;
    }

    public void addVertex(V state) {
        if (valIndexMap.containsKey(state)) return;

        valIndexMap.put(state, adjacencyMap.size());
        //yeah i know
        adjacencyMap.add(Arrays.asList(((T[]) Array.newInstance(transitionType, adjacencyMap.size() + 1))));

        for (int i = 0; i < adjacencyMap.size() - 1; i++) {
            adjacencyMap.get(i).add(null);
        }
    }

    public void addEdge(T transition) {
        if (getEdge(transition.getStartValue(), transition.getEndValue()) != null) return;

        setDirectionalEdge(transition);
    }

    public void setDirectionalEdge(T transition) {
        T at = getEdge(transition.getStartValue(), transition.getEndValue());

        transitions.remove(at);
        transitions.add(transition);
        adjacencyMap.get(valIndexMap.get(transition.getStartValue())).set(valIndexMap.get(transition.getEndValue()), transition);
    }

    public T getEdge(V start, V end) {
        //TODO: fix this bad error :)
        if (!valIndexMap.containsKey(start) || !valIndexMap.containsKey(end)) throw new RuntimeException("start or end state no existy existy");

        //omg .get .get .get .get aaaaaaaaaaaaaaaaaaaaa
        return adjacencyMap.get(valIndexMap.get(start)).get(valIndexMap.get(end));
    }

    public Set<V> getVertices() {
        return valIndexMap.keySet();
    }

    public List<T> getAllEdges() {
        return transitions;
    }
    public List<T> getEdges(V state, EdgeType t) {
        ArrayList<T> outgoing = new ArrayList<>();
        ArrayList<T> incoming = new ArrayList<>();

        if (!valIndexMap.containsKey(state)) return null;

        int j = valIndexMap.get(state);

        for (int i = 0; i < adjacencyMap.size(); i++) {
            outgoing.add(adjacencyMap.get(i).get(j));
            incoming.add(adjacencyMap.get(j).get(i));
        }

        switch (t) {
            case Incoming:
                return incoming;
            case Outgoing:
                return outgoing;
            default:
                return Stream.concat(incoming.stream(), outgoing.stream()).collect(Collectors.toList());
        }
    }
}
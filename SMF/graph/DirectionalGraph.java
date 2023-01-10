package frc.robot.ShamLib.SMF.graph;

import frc.robot.ShamLib.SMF.graph.exceptions.ExistingEdgeException;
import frc.robot.ShamLib.SMF.graph.exceptions.UnfoundVertexException;
import frc.robot.ShamLib.SMF.transitions.TransitionBase;

import java.lang.reflect.Array;
import java.util.*;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class DirectionalGraph<V extends Enum<V>, T extends TransitionBase<V>> {
    private List<List<T>> adjacencyMap;
    private HashMap<V, Integer> valIndexMap;
    private List<T> transitions;
    private Class<?> transitionType;

    private int count;

    public DirectionalGraph(Class<?> type) {
        adjacencyMap = new ArrayList<>();
        valIndexMap = new HashMap<>();

        transitionType = type;
        count = 0;
    }

    public void addVertex(V state) {
        if (valIndexMap.containsKey(state)) return;

        valIndexMap.put(state, count);
        //yeah i know
        adjacencyMap.add(Arrays.asList(((T[]) Array.newInstance(transitionType, count + 1))));

        for (int i = 0; i < adjacencyMap.size() - 1; i++) {
            adjacencyMap.get(i).add(null);
        }

        count++;
    }

    public void addDirectionalEdge(T transition) {
        if (getDirectionalEdge(transition.getStartValue(), transition.getEndValue()) != null) return;

        setDirectionalEdge(transition);
    }

    public void setDirectionalEdge(T transition) {
        T at = getDirectionalEdge(transition.getStartValue(), transition.getEndValue());

        transitions.remove(at);
        transitions.add(transition);
        adjacencyMap.get(valIndexMap.get(transition.getStartValue())).set(valIndexMap.get(transition.getEndValue()), transition);
    }

    public T getDirectionalEdge(V start, V end) {
        //TODO: fix this bad error :)
        if (!valIndexMap.containsKey(start) || !valIndexMap.containsKey(end)) throw new RuntimeException("start or end state no existy existy");

        //omg .get .get .get .get aaaaaaaaaaaaaaaaaaaaa
        return adjacencyMap.get(valIndexMap.get(start)).get(valIndexMap.get(end));
    }

    public Set<V> getVertices() {
        return valIndexMap.keySet();
    }

    public List<T> getEdges() {
        return transitions;
    }
}
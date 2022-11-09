package frc.robot.ShamLib.SMF.graph;

public class DirectionalEdge<V, E, T extends Enum<T>> {
    private Vertex<V, E, T> start;
    private Vertex<V, E, T> end;
    private E value;

    public DirectionalEdge(Vertex<V, E, T> start, Vertex<V, E, T> end, E value) {
        this.start = start;
        this.end = end;
        this.value = value;
    }

    public boolean equals(DirectionalEdge<V, E, T> other) {
        return (getStart() == other.getStart()) && (getEnd() == other.getEnd());
    }

    @Override
    public String toString() {
        return "Start: " + start + ", End: " + end + ", Value: " + value;
    }

    public Vertex<V, E, T> getStart() {
        return start;
    }

    public Vertex<V, E, T> getEnd() {
        return end;
    }

    public E getValue() {
        return value;
    }
}

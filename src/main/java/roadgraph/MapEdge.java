package roadgraph;

public class MapEdge {

    private MapNode start;
    private MapNode end;

    String roadName;
    String roadType;

    double length;

    public MapEdge(MapNode start, MapNode end, String roadName, String roadType, double length) {
        this.start = start;
        this.end = end;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public MapNode getStart() {
        return start;
    }

    public MapNode getEnd() {
        return end;
    }

    public double getLength() {
        return length;
    }
}

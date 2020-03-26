package roadgraph;

import geography.GeographicPoint;

import java.util.HashSet;
import java.util.Set;

public class MapNode {

    private GeographicPoint location;

    private Set<MapEdge> edges;

    public MapNode(GeographicPoint gp) {
        location = gp;
        edges = new HashSet<>();
    }

    public void addEdge(MapNode to, String roadName, String roadType, double length) {
        if (to != null) {
            edges.add(new MapEdge(this, to, roadName, roadType, length));
        }
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public Set<MapEdge> getEdges() {
        return edges;
    }
}

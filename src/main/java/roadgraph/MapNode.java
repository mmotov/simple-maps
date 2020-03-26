package roadgraph;

import geography.GeographicPoint;

import java.util.HashSet;
import java.util.Set;

public class MapNode implements Comparable {

    private GeographicPoint location;

    private Set<MapEdge> edges;

    private double distanceFromStart;

    private double distanceToGoal;

    public MapNode(GeographicPoint gp) {
        location = gp;
        edges = new HashSet<>();
        distanceFromStart = 1.0 / 0.0; // infinity
        distanceToGoal = 0.0;
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

    public void setDistanceFromStart(double distance) {
        distanceFromStart = distance;
    }

    public double getDistanceFromStart() {
        return distanceFromStart;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public void computeDistanceToGoal(MapNode goal) {
        distanceToGoal = location.distance(goal.getLocation());
    }

    @Override
    public int compareTo(Object o) {
        return Double.compare(distanceFromStart + distanceToGoal, ((MapNode)o).getDistanceFromStart());
    }
}

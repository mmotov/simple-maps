/**
 * @author UCSD MOOC development team and YOU
 * <p>
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * <p>
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
public class MapGraph {
    private Map<GeographicPoint, MapNode> adjListMap;
    private int numVertices;
    private int numEdges;


    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        adjListMap = new HashMap<>();
        numVertices = 0;
        numEdges = 0;
    }

    public static void main(String[] args) {
//        MapGraph theMap = new MapGraph();
//        System.out.print("DONE. \nLoading the map...");
//        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
//        System.out.println("DONE.");
//
//        GeographicPoint start = new GeographicPoint(1.0, 1.0);
//        GeographicPoint end = new GeographicPoint(8.0, -1.0);
//
//        List<GeographicPoint> route = theMap.dijkstra(start,end);
//        System.out.println(route);
//        List<GeographicPoint> route2 = theMap.aStarSearch(start, end);
//        System.out.println(route2);

        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
        System.out.println("DONE.");

        GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
        GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

//        List<GeographicPoint> route = theMap.dijkstra(start,end);
        List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

    }

    /**
     * Get the number of vertices (road intersections) in the graph
     *
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        return numVertices;
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        return adjListMap.keySet();
    }

    /**
     * Get the number of road segments in the graph
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        return numEdges;
    }

    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        if (!adjListMap.containsKey(location) && location != null) {
            adjListMap.put(location, new MapNode(location));
            numVertices++;
            return true;
        }
        return false;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length   The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {

        if (!adjListMap.containsKey(from) || !adjListMap.containsKey(to)
                || roadName == null || roadType == null || length < 0) {
            throw new IllegalArgumentException();
        }
        adjListMap.get(from).addEdge(adjListMap.get(to), roadName, roadType, length);
        numEdges++;
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        Queue<MapNode> queue = new LinkedList<>();
        HashSet<MapNode> visited = new HashSet<>();
        HashMap<MapNode, MapNode> parent = new HashMap<>();

        queue.add(adjListMap.get(start));
        visited.add(adjListMap.get(start));

        while (!queue.isEmpty()) {
            MapNode curr = queue.poll();
            if (curr.equals(adjListMap.get(goal))) {
                return getPath(adjListMap.get(start), adjListMap.get(goal), parent);
            }
            Set<MapEdge> edges = curr.getEdges();
            for (MapEdge edge : edges) {
                MapNode neighbor = edge.getEnd();
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    parent.put(neighbor, curr);
                    queue.add(neighbor);
                }
            }
        }
        return null;
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        HashSet<MapNode> visited = new HashSet<>();
        HashMap<MapNode, MapNode> parent = new HashMap<>();
        PriorityQueue<MapNode> queue = new PriorityQueue<>();

        adjListMap.get(start).setDistanceFromStart(0);
        queue.add(adjListMap.get(start));

        int nodesVisited = 0;

        while (!queue.isEmpty()) {
            MapNode curr = queue.poll();
            nodeSearched.accept(curr.getLocation());

            nodesVisited++;

            if (!visited.contains(curr)) {
                visited.add(curr);

                if (curr.equals(adjListMap.get(goal))) {
                    System.out.println("Dijkstra queue deletions: " + nodesVisited);
                    return getPath(adjListMap.get(start), adjListMap.get(goal), parent);
                }

                Set<MapEdge> edges = curr.getEdges();
                for (MapEdge edge : edges) {
                    MapNode neighbor = edge.getEnd();
                    if (!visited.contains(neighbor)) {

                        if (curr.getDistanceFromStart() + edge.getLength() < neighbor.getDistanceFromStart()) {
                            neighbor.setDistanceFromStart(curr.getDistanceFromStart() + edge.getLength());
                            parent.put(neighbor, curr);
                            queue.add(neighbor);
                        }
                    }
                }
            }
        }
        return null;
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        HashSet<MapNode> visited = new HashSet<>();
        HashMap<MapNode, MapNode> parent = new HashMap<>();
        PriorityQueue<MapNode> queue = new PriorityQueue<>();

        MapNode startNode = adjListMap.get(start);
        MapNode goalNode = adjListMap.get(goal);


        startNode.setDistanceFromStart(0);
        startNode.computeDistanceToGoal(startNode);
        queue.add(startNode);

        int nodesVisited = 0;

        while (!queue.isEmpty()) {
            MapNode curr = queue.poll();
            nodeSearched.accept(curr.getLocation());
            if (!visited.contains(curr)) {
                visited.add(curr);
                nodesVisited++;

                if (curr.equals(adjListMap.get(goal))) {
                    System.out.println("A-star queue deletions: " + nodesVisited);
                    return getPath(adjListMap.get(start), adjListMap.get(goal), parent);
                }

                Set<MapEdge> edges = curr.getEdges();
                for (MapEdge edge : edges) {
                    MapNode neighbor = edge.getEnd();
                    if (!visited.contains(neighbor)) {

                        double currWholeDistance = curr.getDistanceFromStart() + curr.getDistanceToGoal() + edge.getLength();
                        double neighborWholeDistance = neighbor.getDistanceFromStart() + neighbor.getDistanceToGoal();
                        if (currWholeDistance < neighborWholeDistance) {
                            neighbor.computeDistanceToGoal(goalNode);
                            neighbor.setDistanceFromStart(curr.getDistanceFromStart() + edge.getLength());
                            parent.put(neighbor, curr);
                            queue.add(neighbor);
                        }
                    }
                }
            }
        }
        return null;
    }

    private List<GeographicPoint> getPath(MapNode start, MapNode goal,
                                          Map<MapNode, MapNode> parent) {
        System.out.println("Building Path");
        List<GeographicPoint> path = new LinkedList<>();
        path.add(0, goal.getLocation());
        MapNode curr = goal;
        do {
            path.add(0, parent.get(curr).getLocation());
            curr = parent.get(curr);
        } while (!curr.equals(start));
        return path;
    }
}

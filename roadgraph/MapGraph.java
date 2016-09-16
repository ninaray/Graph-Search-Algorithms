/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import java.util.HashMap;
import java.util.HashSet;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.PriorityQueue;
import java.util.Comparator;


import geography.GeographicPoint;
import util.GraphLoader;

/**
 * ********IMPLEMENTED BY NINA RAY
 * 
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	
	int numVertices;
	int numEdges;
	private HashMap<GeographicPoint, MapNode> graph;
	private ArrayList<MapEdge> edges;
	
	/** 
	 * ********IMPLEMENTED BY NINA RAY
	 * 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()	{
		// TODO: Implement in this constructor in WEEK 2
		
		numVertices = 0;
		numEdges = 0;
		graph = new HashMap<GeographicPoint, MapNode>();
		edges = new ArrayList<MapEdge>();
	}
	
	/**
	 * ********IMPLEMENTED BY NINA RAY
	 * 
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numVertices;
	}
	
	/**
	 * ********IMPLEMENTED BY NINA RAY
	 * 
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		//TODO: Implement this method in WEEK 2
		
		Set<GeographicPoint> vertices = new HashSet<GeographicPoint>();
		
		if (graph.size() != 0) {
			for (GeographicPoint p: graph.keySet()) {
				vertices.add(p);
			}
		}
		
		return vertices;
	}
	
	/**
	 * ********IMPLEMENTED BY NINA RAY
	 * 
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		//TODO: Implement this method in WEEK 2
		return numEdges;
	}

	
	
	/** 
	 * ********IMPLEMENTED BY NINA RAY
	 * 
	 * Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		// TODO: Implement this method in WEEK 2
		
		if (graph.containsKey(location) == true){
			return false;
		}
		if (location == null) return false;
		
		else {
			MapNode node = new MapNode(location);
			graph.put(location, node);
			numVertices++;
			return true;
		}
	}
	
	/**
	 * ********IMPLEMENTED BY NINA RAY
	 *
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		//TODO: Implement this method in WEEK 2
	
		if (length < 0) throw new IllegalArgumentException();
		if (graph.containsKey(from) == false || graph.containsKey(to) == false)
			throw new IllegalArgumentException();
		if (from == null || to == null || roadName == null || roadType == null) 
			throw new IllegalArgumentException();
		
		MapNode fromnode = graph.get(from);
		MapNode tonode = graph.get(to);
		
		MapEdge edge = new MapEdge(fromnode, tonode, roadName, roadType, length);
		
		edges.add(edge);
		fromnode.neighbors.put(tonode, edge);
		numEdges++;
		
		
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/**
	 * ********IMPLEMENTED BY NINA RAY
	 * 
	 *  Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		Queue<MapNode> queue = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		//Key is current node, Object is parent
		HashMap<GeographicPoint, GeographicPoint> path = new HashMap<GeographicPoint, GeographicPoint>();
		List<GeographicPoint> p = new ArrayList<GeographicPoint>();
		
		MapNode startnode = graph.get(start);
		MapNode goalnode = graph.get(goal);
		
		queue.add(startnode);
		if (!visited.contains(startnode)) visited.add(startnode);
		
		while (!queue.isEmpty()) {
			MapNode current = queue.poll();		
			if (current == goalnode) {
				p = createPath(goal, start, path);
				return p;
			}
			for (MapNode n : current.neighbors.keySet()){
				if (!visited.contains(n)) {
					visited.add(n);
					path.put(n.location, current.location);
					queue.add(n);
					// Hook for visualization.  See writeup.
					nodeSearched.accept(current.location);
				}
			}
			
		}
		
		return null;
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
	}
	
	public List<GeographicPoint> createPath(GeographicPoint goal, GeographicPoint start,
			HashMap<GeographicPoint, GeographicPoint> path) {
		
		List<GeographicPoint> list = new ArrayList<GeographicPoint>();
		
		GeographicPoint point = goal;
		
		while (point != null) {
			list.add(0, point);
			GeographicPoint parent = path.get(point);
			point = parent;
			
		}
				
		return list; 
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** ********IMPLEMENTED BY NINA RAY
	 * 
	 * Traverse the list of locations to create a path from start to goal after using
	 * 	Dijkstra's algorithm
	 * 
	 * @param goal The goal location
	 * @param start The starting location
	 * @param path The list of locations associated with their previously traversed locations
	 * @return The list of intersections that for, the shortest path from start to goal,
	 * 			including start and goal
	 */
	public List<GeographicPoint> djikstraPath(GeographicPoint goal, GeographicPoint start,
			HashMap<GeographicPoint, GeographicPoint> path) {
		
		LinkedList<GeographicPoint> list = new LinkedList<GeographicPoint>();
		
		GeographicPoint point = goal;
		
		while (point != null) {
			list.addFirst(point);
			point = path.get(point);
			
		}
				
		return list; 
	}
	
	/** 
	 * ********IMPLEMENTED BY NINA RAY
	 * 
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		//Initialize Priority Queue
		Comparator<MapNode> comparator = new MapNodeComparator();
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(comparator);
		//Initialize visited set
		HashSet<MapNode> visited = new HashSet<MapNode>();
		//Initialize Parent map (to retrace path)
		HashMap<GeographicPoint, GeographicPoint> path = new HashMap<GeographicPoint, GeographicPoint>();
		//Initialize shortest path
		List<GeographicPoint> p = new ArrayList<GeographicPoint>();
		
		//Initialize distances to infinity
		for (MapNode m : graph.values()) {
			m.setPriorityDistance(Integer.MAX_VALUE);
		}
		
		MapNode startnode = graph.get(start);
		MapNode goalnode = graph.get(goal);
		
		startnode.setPriorityDistance(0);
		queue.add(startnode);
		//path.put(startnode.location, startnode.location);
		
		
		while (!queue.isEmpty()) {
			MapNode current = queue.poll();	

			nodeSearched.accept(current.location);
			if (!visited.contains(current)) {
				visited.add(current);
				
				if (current == goalnode) {					
					p = djikstraPath(goal, start, path);
					break;
				}
				for (MapNode n : current.neighbors.keySet()){
					double edgeweight = current.neighbors.get(n).getDistance();
										
					//If the path through the current node is shorter than n's distance...
					if (n.getPriorityDistance() > current.getPriorityDistance() + edgeweight) {
						n.setPriorityDistance(current.getPriorityDistance() + edgeweight);
						path.put(n.location, current.location);
						
						queue.add(n);
					}
				}
			}
			
		}
		if (!p.isEmpty()) { 
			
			//for (int i = 0; i < p.size(); i++) {
			//	System.out.println(p.get(i).toString());
			//}
			return p;
			
		}
		
		else {
			return null;
		}
	}

	/**
	 * ********IMPLEMENTED BY NINA RAY 
	 * 
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** 
	 * ********IMPLEMENTED BY NINA RAY
	 * 
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		//Initialize Priority Queue
		Comparator<MapNode> comparator = new MapNodeComparator();
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(comparator);
		//Initialize visited set
		HashSet<MapNode> visited = new HashSet<MapNode>();
		//Initialize Parent map (to retrace path)
		HashMap<GeographicPoint, GeographicPoint> path = new HashMap<GeographicPoint, GeographicPoint>();
		//Initialize shortest path
		List<GeographicPoint> p = new ArrayList<GeographicPoint>();
		
		//Initialize distances to infinity
		for (MapNode m : graph.values()) {
			m.setPriorityDistance(Integer.MAX_VALUE);
			m.setFinalDistance(0);
		}
		
		MapNode startnode = graph.get(start);
		MapNode goalnode = graph.get(goal);
		
		startnode.setPriorityDistance(0);
		startnode.setFinalDistance(Integer.MAX_VALUE);
		queue.add(startnode);
		
		//path.put(startnode.location, startnode.location);
		
		
		while (!queue.isEmpty()) {
			MapNode current = queue.poll();	

			nodeSearched.accept(current.location);
			
			if (!visited.contains(current)) {
				visited.add(current);
				
				if (current == goalnode) {					
					p = djikstraPath(goal, start, path);
					break;
				}
				for (MapNode n : current.neighbors.keySet()){
					double edgeweight = current.neighbors.get(n).getDistance();
					double finalweight = n.location.distance(goal);
					
					//If the path through the current node is shorter than n's distance...
					if (finalweight <= current.location.distance(goal)) {
						
						n.setFinalDistance(finalweight);
						n.setPriorityDistance(current.getPriorityDistance() + edgeweight);
						path.put(n.location, current.location);
						
						queue.add(n);
					}					
				}
			}
			
		}
		if (!p.isEmpty()) { 
			return p;
		}
		
		else {
			return null;
		}
	}

	
	
	public static void main(String[] args)
	{
		//System.out.print("Making a new map...");
		//MapGraph theMap = new MapGraph();
		//System.out.print("DONE. \nLoading the map...");
		//GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		//GeographicPoint start = new GeographicPoint(1.0, 1.0);
		//GeographicPoint end = new GeographicPoint(8.0, -1.0);
		//theMap.dijkstra(start, end);
		//System.out.println("DONE.");
		
		// You can use this method for testing.  
		
        MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}

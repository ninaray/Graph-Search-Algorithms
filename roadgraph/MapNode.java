package roadgraph;

import geography.GeographicPoint;
import java.util.HashMap;

/** A class which represents a vertex on a MapGraph.
 * *
 */
public class MapNode{
	GeographicPoint location;
	private double prioritydistance = 0; //Distance from start node to this node
	private double finaldistance = Integer.MAX_VALUE; //Distance from end node to this node
	
	HashMap<MapNode, MapEdge> neighbors = new HashMap<MapNode, MapEdge>();
	
	/** Creates an instance of MazeNode, with location represented by 
	 * GeographicPoint location, and an empty list of neighbors.
	 */
	public MapNode(GeographicPoint location) {
		this.location = location;
	}
	
	public double getPriorityDistance() {
		return prioritydistance;
	}
	
	public void setPriorityDistance(double d) {
		this.prioritydistance = d;
	}
	
	public double getFinalDistance() {
		return finaldistance;
	}
	
	public void setFinalDistance(double f) {
		this.finaldistance = f;
	}
	
}

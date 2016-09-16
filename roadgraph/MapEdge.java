package roadgraph;

/** A class which represents an edge between two vertices on a MapGraph.
 * *
 */
public class MapEdge {
	
	private MapNode start;
	private MapNode end;
	private String roadName;
	private String roadType;
	private double distance;
	
	/** Create a new instance of MazeEdge with directed edge from
	 * MazeNode start to MazeNode end that are already in the graph - 
	 * this represents a road in the map that connects two intersections.
	 * Road name is represented by String roadName and type is represented
	 * by String roadType. The length of the road is represented by int distance.
	 */
	public MapEdge(MapNode start, MapNode end, 
			String roadName, String roadType, double distance) {
		this.start = start;
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.distance = distance;
		
	}
	
	public MapNode getStart() {
		return start;
	}
	
	public MapNode getEnd() {
		return end;
	}
	
	public String getRoadName() {
		return roadName;
	}

	public String getRoadType() {
		return roadType;
	}
	
	public double getDistance() {
		return distance;
	}
	
}

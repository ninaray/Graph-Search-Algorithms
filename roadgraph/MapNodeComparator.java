package roadgraph;

import java.util.Comparator;

public class MapNodeComparator implements Comparator<MapNode> {
	
	public int compare(MapNode curr, MapNode other) {
		if (curr.getPriorityDistance() < other.getPriorityDistance()) {
			return -1;
		}
		if (curr.getPriorityDistance() > other.getPriorityDistance()) {
			return 1;
		}
		else {
			return 0;
		}
	}
}

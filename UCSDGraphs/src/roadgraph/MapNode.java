package roadgraph;

import java.util.List;
import java.util.LinkedList;
import geography.GeographicPoint;

/**
 * MapNode Class
 * @author gregorymankes
 * 
 * This class serves as a container for both the location and edges field associated 
 * with a MapNode. Includes methods for creating edges between two nodes and retrieval
 * of this data.
 */
public class MapNode {
	
	// used as our X and Y coordinates.
	private GeographicPoint gp;
	
	// Keep track here of what our neighbors are
	private List<MapEdge> edges;
	
	private double distanceFromStart;
	
	private double distanceFromGoal;
	
	
	/**
	 * Constructor that merely sets up the node for adding edges later
	 * @param gp
	 */
	MapNode(GeographicPoint gp){
		this.gp = gp;
		this.edges = new LinkedList<MapEdge>();
		this.distanceFromStart = 0;
		this.distanceFromGoal = 0;
	}
	
	/**
	 * Constructor thats sets up the node with neighboring edge
	 * @param gp the location of this node
	 * @param neighbor the location of the next node
	 * @param streetName the name of the street
	 * @param streetType the type of the street
	 */
	MapNode(GeographicPoint gp, GeographicPoint neighbor, String streetName,
			String streetType){
		this(gp);
		addNeighbor(neighbor, streetName, streetType);
	}
	
	/**
	 * Adds an edge for keeping track of what nodes neighbor this one. 
	 * @param neighbor the location of the node
	 * @param streetName the name of the street
	 * @param streetType the type of the street
	 */
	void addNeighbor(GeographicPoint neighbor, String streetName, String streetType){
		this.edges.add(new MapEdge(this.gp, neighbor, streetName, streetType));
	}
	
	void addNeighbor(GeographicPoint neighbor, String streetName, String streetType,
			double length){
		this.edges.add(new MapEdge(this.gp, neighbor, streetName, streetType,length));
	}
	
	/**
	 * Gets the neighbors as a list of edges coming from this node
	 * @return the list of edges
	 */
	List<MapEdge> getNeighbors(){
		return this.edges;
	}
	
	public void setDistanceFromStart(double distance){
		this.distanceFromStart = distance;
	}
	
	public double getDistanceFromStart(){
		return this.distanceFromStart;
	}
	
	public void setDistanceFromGoal(double distance){
		this.distanceFromGoal = distance;
	}
	
	public double getDistanceFromGoal(){
		return this.distanceFromGoal;
	}
	
	public double getFn(){
		return this.getDistanceFromStart()+this.getDistanceFromGoal();
	}
	
	public GeographicPoint getLocation(){
		return this.gp;
	}
	
	/**Prints the node and the out edges that it has
	 * 
	 */
	@Override
	public String toString(){
		//: Need to finish this later
		String toReturn = "This node is "+gp.getX()+", "+gp.getY()+". Its edges are: \n";
		for(MapEdge me : this.edges){
			toReturn+=me.debugString();
		}
		return toReturn;
	}
}

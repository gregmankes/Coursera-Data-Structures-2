package roadgraph;

import geography.GeographicPoint;
/**
 * MapEdge Class
 * @author gregorymankes
 *
 * This class serves as a container for all data associated with an edge between two
 * vertices in a map graph.
 */
public class MapEdge {
	// needed for locations of starting and ending vertex
	private GeographicPoint start;
	private GeographicPoint end;
	
	//data for keeping track of edges on map
	private String streetName;
	private String streetType;
	
	// data for keeping track of fastest routes in later weeks
	private double distance;
	
	
	/**
	 * Constructor for an edge. 
	 * @param start the start location of the edge
	 * @param end the end location of the edge
	 * @param streetName the street name
	 * @param streetType the street type
	 */
	MapEdge(GeographicPoint start, GeographicPoint end, String streetName,
			String streetType){
		this.start = start;
		this.end = end;
		this.streetName = streetName;
		this.streetType = streetType;
		this.distance = start.distance(end);
	}
	
	/**
	 * The main constructor with all of the fields
	 * @param start the start location
	 * @param end the end location
	 * @param streetName the street name
	 * @param streetType the street type
	 * @param length
	 */
	MapEdge(GeographicPoint start, GeographicPoint end, String streetName,
			String streetType, double length){
		this.start = start;
		this.end = end;
		this.streetName = streetName;
		this.streetType = streetType;
		this.distance = length;
	}
	
	public GeographicPoint getStart(){
		return this.start;
	}
	
	public GeographicPoint getEnd(){
		return this.end;
	}
	
	public String getStreetName(){
		return this.streetName;
	}
	
	public String getStreetType(){
		return this.streetType;
	}
	
	public double getDistance(){
		return this.distance;
	}
	
	/**
	 * toString override. just prints out all of the data.
	 */
	@Override
	public String toString(){
		String toReturn = "Starting location:\n"+this.start.toString()+
		"\nEnding Location\n"+this.end.toString()+
		"\nStreet Type:\n"+this.getStreetType()+
		"\nStreet Nane:\n"+this.getStreetName()+
		"\nDistance:\n"+this.getDistance();
		return toReturn;
	}
	
	/**Prints the edge data in map form
	 * 
	 * @return the debug string
	 */
	public String debugString(){
		String toReturn = ""+this.start.getX()+" "+this.start.getY()+" "+
				this.end.getX()+" "+this.end.getY()+" "+this.getStreetName()+" "+
				this.getStreetType()+"\n";
		
		return toReturn;
	}
}

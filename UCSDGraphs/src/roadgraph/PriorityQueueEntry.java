package roadgraph;

import geography.GeographicPoint;

/**PriorityQueueEntry
 * This class serves as a carriage for all of the fields necessary to be placed
 * into a priorityQueue
 * 
 * @author gregorymankes
 *
 */
public class PriorityQueueEntry implements Comparable<PriorityQueueEntry>{
	
	private double priority;
	
	/**
	 * The location
	 */
	private GeographicPoint gp;
	
	/**Constructor for PriorityQueueEntry
	 * 
	 * @param gp the geographic point location
	 * @param startDistance the distance from the start
	 * @param endDistance the distance from the end
	 */
	public PriorityQueueEntry(GeographicPoint gp, double priority){
		this.gp = gp;
		this.priority = priority;
	}
	
	/**get f(n)
	 * 
	 * @return the value of f(n)
	 */
	public double getPriority(){
		return this.priority;
	}
	
	public GeographicPoint getLocation(){
		return this.gp;
	}
	
	public void setPriority(double priority){
		this.priority = priority;
	}
	
	/**CompareTo Method
	 * Used in priorityQueue to sort the entries.
	 * Returns an int that determines the ordering.
	 */
	public int compareTo(PriorityQueueEntry other){
		return (int)((this.getPriority() - other.getPriority())*1000);
	}
	
	/** Overrides toString method. Prints out all of the data
	 * used for debugging
	 */
	public String toString(){
		String toReturn = "Location: "+this.gp.getX()+", "+this.gp.getY()+"\n"+
				"Priority: "+this.getPriority();
		return toReturn;
	}
	
}

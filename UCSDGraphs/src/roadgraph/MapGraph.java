/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	HashMap<GeographicPoint,MapNode> nodes;
	
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		nodes = new HashMap<GeographicPoint,MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		// The number of vertices is equal to the number of locations
		// (GeographicPoints) in the HashMap
		return nodes.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		// This is the set of keys in the map, which in this case
		// is are the GeographicPoints
		return nodes.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		
		// Get the keys from the map
		Set<GeographicPoint> keys = getVertices();
		
		// Initialize a count
		int count = 0;
		
		// Iterate over all of the keys, punching them into the
		// HashMap. Then get the number of edges from each MapNode.
		// Increment the count for each nodes neighbor size.
		MapNode current = null;
		for(GeographicPoint gp : keys){
			current = nodes.get(gp);
			count+=current.getNeighbors().size();
		}
		return count;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// if the param is null or it is already in the map, return false
		if (nodes.containsKey(location) || location == null){
			return false;
		}
		// otherwise return true and place it in the map
		nodes.put(location, new MapNode(location));
		return true;
	}
	
	/**
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

		// if any of the parameters are null, throw an exception
		if (from == null || to == null || roadName == null || roadType == null ||
				length < 0){
			throw new IllegalArgumentException("One or more of the arguments of"
					+ "addEdge are null");
		}
		
		// if the two locations have not yet been added to the map
		// throw an exception
		if(!nodes.containsKey(from) || !nodes.containsKey(to)){
			throw new IllegalArgumentException("From or to has not yet been added"
					+ "to the map");
		}
		
		nodes.get(from).addNeighbor(to, roadName, roadType, length);
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
	
	/** Find the path from start to goal using breadth first search
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
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		// Init the list here so that we can pass it in to the construct path
		// subroutine.
		HashMap<GeographicPoint,GeographicPoint> parentMap = 
				new HashMap<GeographicPoint,GeographicPoint>();
		
		// Search through the map from the start to the goal, adding the parents 
		// along the way
		boolean found = bfsSearch(start, goal, nodeSearched, parentMap);
		
		// if we didn't find it, return an empty linkedlist
		if(!found){
			System.out.println("No path exists");
//			return new LinkedList<GeographicPoint>();
			return null;
		}
		
		// reconstruct the path and return it as a list
		return constructPath(start, goal, parentMap);
	}
	
	/** Construct the path from start to finish from the parentMap
	 * 
	 * @param start the start location
	 * @param goal the end location
	 * @param parentMap the parent node path
	 * @return the path in list form
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, 
			GeographicPoint goal,HashMap<GeographicPoint, GeographicPoint> parentMap){
		
		// Initialize the path and set the goal to be the current node
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint current = goal;
		
		// Walk through the parent map, adding the parents along the way to
		// the path
		while(current.distance(start) > 0){
			// since we are starting from the goal to the start, 
			// there is only one shortest path 
			path.addFirst(current);
			current = parentMap.get(current);
		}
		
		// add the start to the path
		path.addFirst(start);
		
		// return the path
		return path;
	}
	
	/**Factoring Search into separate subroutine
	 * 
	 * @param start the start location
	 * @param goal the end location
	 * @param nodeSearched the consumer hook for visualization
	 * @param parentMap the map of the path of nodes visited
	 * @return
	 */
	private boolean bfsSearch(GeographicPoint start,
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched, 
			HashMap<GeographicPoint,GeographicPoint> parentMap){
		
		// Initialize the visited set and the FIFO queue
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		LinkedList<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		
		
		// enqueue start location and add to visited and report start to Consumer
		queue.add(start);
		visited.add(start);
		nodeSearched.accept(start);
		
		// Init outside of loop
		GeographicPoint currentNode;
		
		// while the queue is not empty
		while(!queue.isEmpty()){
			// remove the first in line
			currentNode = queue.removeFirst();
			
			// check against the goal
			if(currentNode.distance(goal) == 0){
				return true;
			}
			
			// Get the list of edges from the current node
			List<MapEdge> neighbors = nodes.get(currentNode).getNeighbors();
			
			// for each of the neighbors not in the visited set
			for(MapEdge me : neighbors){
				if(!visited.contains(me.getEnd())){
					// add end GeographicPoint to visited
					visited.add(me.getEnd());
					// put the currentNode as the end GeographicPoint's parent
					parentMap.put(me.getEnd(), currentNode);
					// enqueue the neighbor to the queue
					queue.addLast(me.getEnd());
					// add this to the consumer
					nodeSearched.accept(me.getEnd());
				}
			}
		}
		// if we got here, we didn't find it
		return false;
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
	
	/** Find the path from start to goal using Dijkstra's algorithm
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

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
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
	
	/** Find the path from start to goal using A-Star search
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
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/**Prints each nodes directed edges. Used for debugging.
	 * 
	 */
	void printGraph(){
		// Create the key entry set so we can get all of the Geographic points
		// and enter them into the map.
		Set<GeographicPoint> nodeList = this.nodes.keySet();
		
		// Iterate over the Geographic points and print out the corresponding
		// MapNode's toString method.
		for(GeographicPoint n : nodeList){
			System.out.println(nodes.get(n).toString());
		}
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
//		theMap.printGraph();
		
		GeographicPoint start = new GeographicPoint(1,1);
		GeographicPoint goal = new GeographicPoint(8,-1);

		List<GeographicPoint> route = theMap.bfs(start, goal);

		System.out.println(route.toString());
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}

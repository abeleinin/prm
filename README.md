# Probabilistic Roadmap (PRM)

Implementation of the [probabilistic roadmap algorithm](https://en.wikipedia.org/wiki/Probabilistic_roadmaphttps://en.wikipedia.org/wiki/Probabilistic_roadmap) in ROS for path planning.

## Rviz Environment

![PRM Rviz image](https://github.com/abeleinin/prm/blob/main/examples/rviz_environment.png?raw=true)

## Approach 

1. Generate n random points in a given environment.
2. Remove points that collide with objects.
3. Create every node with it's k nearest neighbors.
4. Remove edges that collide with objects.
5. Execute a shortest path algorithm (like Dijkstra's algorithm) to determine the shortest path to the goal position.

## Example

### n = 3000 and k = 5 map

![PRM Rviz image](https://github.com/abeleinin/prm/blob/main/examples/PRM_n3000_k5.png?raw=true)

## References 

- Referenced [KaleabTessera/PRM-Path-Planning](https://github.com/KaleabTessera/PRM-Path-Planning)
- Starter code provide through affiliation with [VAIL](https://vail.sice.indiana.edu/)



from math import sqrt, pow
from typing import List, Tuple
from heapq import heappop, heappush
from helpers import Map


def shortest_path(map_data: Map, start_node: int, end_node: int) -> List[int]:
    """Get the shortest path between a start and end node using the A* algorithm.
        Assumption: we are dealing only with straight connections, i.e. also the road distances are based on the straight distance.
        Estimate: straight line distance, what is admissible as it never overestimates and is optimistic given a 2D plane.

        Parameters
        ----------
        map_data : Map, required
            The Map data.
        start_node : int, required
            The start node id.
        end_node : int, required
            The end node id.

        Returns
        ----------
        list
            Returns the list with path as node ids in order.
    """

    # Validate & Prepare
    if not isinstance(map_data, Map) or not isinstance(start_node, int) or not isinstance(end_node, int):
        raise ValueError("Please provide correct input...")
    if start_node == end_node:
        print("Start Node is End Node....")
        return [start_node]

    map_intersections: dict = map_data.intersections
    if not map_intersections or start_node not in map_intersections or end_node not in map_intersections:
        print("Start or End Node not in intersections....")
        return []
    map_roads: List[List[int]] = map_data.roads

    # (<node id>)
    explored_nodes: set = set()
    # {<node id>: <distance>}
    straight_line_distances: dict = {}
    # {"<start node id>-<end node id>": <distance>}
    road_distances: dict = {}
    # [(<total distance up to the point>, <road distance to node>, <node id>, <list with path to node>)]
    frontier_distances_path: List[Tuple(int, int, int, List[int])] = []
    # [(<total distance to end node>, <list with path to end node>)]
    possible_result_paths: List[Tuple(int, List[int])] = []

    # Process
    start_node_coordinates: List[float] = map_intersections[start_node]
    end_node_coordinates: List[float] = map_intersections[end_node]
    straight_distance: float = calculate_straight_distance(
        start_node_coordinates, end_node_coordinates)
    heappush(frontier_distances_path, (straight_distance, 0, start_node, []))

    current_node: Tuple(int, int, int, List[int]) = None

    while len(frontier_distances_path) > 0:
        current_node = heappop(frontier_distances_path)
        current_node_id: int = current_node[2]
        current_node_neighbors: List[int] = map_roads[current_node_id]

        for current_node_neighbor_id in current_node_neighbors:
            if current_node_neighbor_id not in explored_nodes:
                if current_node_neighbor_id not in straight_line_distances:
                    straight_line_distance_end_node: float = calculate_straight_distance(
                        map_intersections[current_node_neighbor_id], end_node_coordinates)
                    straight_line_distances[current_node_neighbor_id] = straight_line_distance_end_node
                if f"{current_node_neighbor_id}-{current_node_id}" not in road_distances:
                    straight_line_distance_current_to_neighbor_node: float = calculate_straight_distance(
                        map_intersections[current_node_id], map_intersections[current_node_neighbor_id])
                    road_distances[f"{current_node_id}-{current_node_neighbor_id}"] = straight_line_distance_current_to_neighbor_node
                    road_distances[f"{current_node_neighbor_id}-{current_node_id}"] = straight_line_distance_current_to_neighbor_node

                current_road_distance = current_node[1]
                road_distance_current_to_neighbor: int = road_distances[f"{current_node_id}-{current_node_neighbor_id}"]
                total_road_distance_to_neighbor: int = current_road_distance + road_distance_current_to_neighbor
                current_node_neighbor_path: List[int] = current_node[3] + [current_node_id]

                if current_node_neighbor_id == end_node:
                    heappush(possible_result_paths, (total_road_distance_to_neighbor, current_node_neighbor_path))
                else:
                    total_distance_with_estimate = total_road_distance_to_neighbor + straight_line_distances[current_node_neighbor_id]
                    heappush(frontier_distances_path, (total_distance_with_estimate, total_road_distance_to_neighbor, current_node_neighbor_id, current_node_neighbor_path))

        explored_nodes.add(current_node_id)

    # Return result
    if len(possible_result_paths) == 0:
        return []
    else:
        return heappop(possible_result_paths)[1] + [end_node]


def calculate_straight_distance(start_point: List[float], end_point: List[float]) -> float:
    """Calculate straight distance between two point in a 2D coordinate system.
        Based on the Pythagorean theorem.

        Parameters
        ----------
        start_point : List[float], required
            List with the x and y coordinate of the start point.
        end_point : List[float], required
            List with the x and y coordinate of the end point.

        Returns
        ----------
        float
            Returns the straight distance.
    """

    x1: float = start_point[0]
    y1: float = start_point[1]
    x2: float = end_point[0]
    y2: float = end_point[1]

    distance: float = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0)

    return distance

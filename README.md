## Educational path search 

This repository contains an educational path search algorithm, just for studying. 

1. **Basic Path Search:** The purpose of basic path search is to find a possible (not necessarily the best) path between two points on a grid cell. If there is at least a path between two poins, this algorithm will find it anyway; however the approach is not efficient. In other words, it always works, but it is not the most efficient way. 
 
 
2. **A-star Search:** This approach is probably the most efficient way to fond the best path between two points. However, one drawback is that unlike the **basic search** (defined above), it may not be able to find the path in the first forward search and we may need to try different 

The pipeline consists of two main classes `RouteNode` and `RouteFinder`. 

- Class `RouteNode` encapsulates the posistion of the root node (`pos`), the grid cell indices of the neighbors `neighbor_indices`, and the costs associated with reaching from the root node to its neighbors. 

- Class `RouteFinder` contains the following variables and methods: 
 
  - **Variables**: 
    - `route_nodes`: a list of `RouteNode` objects 
    - `delta_names`: a working (temporary) matrix, with the same size as the `grid_cells`, containing the direction arrows (<, >, ^, V) 
    - `directions`: the same matrix as `delta_names`, but it contains the final direction arrows, extracted from the `delta_names` based on the search approach 
    - `count_table`: an integer matrix, with the same size as the `grid_cells`, containing the number of pipeline iteration while exploring each cell 
    - `search_counter`: a cumulative integer counter, keeping the cumulative number of pipeline iteration while exploring each cell 
        
  - **Main Functions**: 
    - `extract_path_indices`: to summarize and extract the final output out of the search method selected 
    - `search_basic`: the basic search algorithm to find a path between two points on the map. As long as a path exists between two cells, this fucntion finds an existing path anyway, but it is not the most efficient way. 
    - `search_a_star`: the A*-based search algorithm to find a path between two points on the map.

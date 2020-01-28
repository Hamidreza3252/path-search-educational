    def find_neighbor_indices_basic(self, pos, grid_cells, modify_grid_cells):
        
        # neighbor_poses = []

        neighbor_indices = []
        costs = []
        
        (up_neighbor_indices, down_neighbor_indices, left_neighbor_indices, right_neighbor_indices) = RouteFinder.get_neighbors_indices(pos, grid_cells)
        
        # index = 0
        # routs_count = 0
        
        # unique_index_id = 0
        # self.neighbors_indices_table
        
        if ( (pos != up_neighbor_indices) and 
             (grid_cells[up_neighbor_indices[0], up_neighbor_indices[1]] == 0)):
            
            if (modify_grid_cells):
                self.search_counter += 1
                grid_cells[up_neighbor_indices[0], up_neighbor_indices[1]] = 1
                self.count_table[up_neighbor_indices[0], up_neighbor_indices[1]] = self.search_counter
                self.delta_names[up_neighbor_indices[0], up_neighbor_indices[1]] = "^"
                # self.cost_table[up_neighbor_indices[0], up_neighbor_indices[1]] = 
                
            neighbor_indices.append([up_neighbor_indices[0], up_neighbor_indices[1]])
            costs.append(1)
            # index += 1

            # if (grid_cells[up_neighbor_indices[0], up_neighbor_indices[1]] == 0):
            #    routs_count += 1
            
        if ( (up_neighbor_indices != down_neighbor_indices) and (pos != down_neighbor_indices) and 
             (grid_cells[down_neighbor_indices[0], down_neighbor_indices[1]] == 0)):

            if (modify_grid_cells):
                self.search_counter += 1
                grid_cells[down_neighbor_indices[0], down_neighbor_indices[1]] = 1
                self.count_table[down_neighbor_indices[0], down_neighbor_indices[1]] = self.search_counter
                self.delta_names[down_neighbor_indices[0], down_neighbor_indices[1]] = "v"
                
            neighbor_indices.append([down_neighbor_indices[0], down_neighbor_indices[1]])
            # delta_names.append("v")
            costs.append(1)
            # index += 1

            # if (grid_cells[down_neighbor_indices[0], down_neighbor_indices[1]] == 0):
            #    routs_count += 1

        if ( (pos != left_neighbor_indices) and 
             (grid_cells[left_neighbor_indices[0], left_neighbor_indices[1]] == 0)):

            if (modify_grid_cells):
                self.search_counter += 1
                grid_cells[left_neighbor_indices[0], left_neighbor_indices[1]] = 1
                self.count_table[left_neighbor_indices[0], left_neighbor_indices[1]] = self.search_counter
                self.delta_names[left_neighbor_indices[0], left_neighbor_indices[1]] = "<"
                
            neighbor_indices.append([left_neighbor_indices[0], left_neighbor_indices[1]])
            # delta_names.append("<")
            costs.append(1)
            # index += 1

            #if (grid_cells[left_neighbor_indices[0], left_neighbor_indices[1]] == 0):
            #    routs_count += 1

        if ( (left_neighbor_indices != right_neighbor_indices) and (pos != right_neighbor_indices) and 
             (grid_cells[right_neighbor_indices[0], right_neighbor_indices[1]] == 0)):

            if (modify_grid_cells):
                self.search_counter += 1
                grid_cells[right_neighbor_indices[0], right_neighbor_indices[1]] = 1
                self.count_table[right_neighbor_indices[0], right_neighbor_indices[1]] = self.search_counter
                self.delta_names[right_neighbor_indices[0], right_neighbor_indices[1]] = ">"
                
            neighbor_indices.append([right_neighbor_indices[0], right_neighbor_indices[1]])
            # delta_names.append(">")
            costs.append(1)
            # index += 1

            # if (grid_cells[right_neighbor_indices[0], right_neighbor_indices[1]] == 0):
            #    routs_count += 1
            
        return (neighbor_indices, costs)
    

    def search_basic(self, grid_cells, init_states, goal_pos, max_depth, cell_blockage_value=-1):
        
        init_pos = [init_states[0], init_states[1]]
        
        if (init_pos == goal_pos):
            return [0.0, goal_pos[0], goal_pos[1]]
        
        if (grid_cells[init_pos[0], init_pos[1]] == 1):
            return [cell_blockage_value, goal_pos[0], goal_pos[1]]
        
        self.reset(grid_cells.shape)
        
        current_pos = init_pos

        branch_search_completed = False
        debug_counter = 0

        grid_cells[init_pos[0], init_pos[1]] = 1
        
        self.search_counter = 0
        self.count_table[init_pos[0], init_pos[1]] = 0

        (neighbor_indices, costs) = self.find_neighbor_indices_basic(current_pos, grid_cells, modify_grid_cells = True)
        # for debug
        # print("current_pos: ", current_pos, "  -  neighbor_indices: ", neighbor_indices)
        new_nodes = []
        current_node = RouteNode(current_pos, neighbor_indices, costs)
        new_nodes.append(current_node)
        self.route_nodes += new_nodes
        
        while (not branch_search_completed):
            # index = 0

            neighbor_nodes = []

            for node in new_nodes:

                for node_pos in node.neighbor_indices:
                    (neighbor_indices, costs) = self.find_neighbor_indices_basic(node_pos, grid_cells, modify_grid_cells = True)
                    
                    # for debug
                    # print("=================================================")
                    # print("node_pos: ", node_pos, "  -  neighbor_indices: ", neighbor_indices)
                    
                    current_node = RouteNode(node_pos, neighbor_indices, costs)
                    neighbor_nodes.append(current_node)
                    
                    for neighbor_pos in neighbor_indices:

                        # print("Hamid - 1", neighbor_pos)

                        if (neighbor_pos == goal_pos):
                            branch_search_completed = True

                            self.route_nodes += neighbor_nodes
                            total_cost = self.extract_path_info(init_pos, goal_pos, grid_cells.shape)
                            
                            # print("found", neighbor_pos)

                            return [total_cost, int(goal_pos[0]), int(goal_pos[1])]

            new_nodes = neighbor_nodes
            self.route_nodes += new_nodes
            
            if (debug_counter == max_depth):
                branch_search_completed = True
            
            debug_counter += 1
        
        return [cell_blockage_value, goal_pos[0], goal_pos[1]]
    



    
    def extract_path_info_OLD(self, init_pos, goal_pos, grid_shape):
        
        pointer_pos = goal_pos.copy()
        # path_delta_names = []
        total_cost = 0.0
        node_index = len(self.route_nodes) - 1
        
        while (pointer_pos != init_pos):
            if (node_index < 0):
                return ([], [], 0.0)
            
            node = self.route_nodes[node_index]
            
            if (pointer_pos in node.neighbor_indices):
                index = node.neighbor_indices.index(pointer_pos)
                total_cost += node.costs[index]
                self.direction_table[node.pos[0], node.pos[1]] = self.delta_names[pointer_pos[0], pointer_pos[1]]
                self.cost_table[node.pos[0], node.pos[1]] = total_cost
                
                # search_key = [key for (key, value) in aa.items() if value == [node.pos[0], node.pos[1]]]
                
                # print("self.cost_table", node.pos, self.cost_table[node.pos[0], node.pos[1]])
                pointer_pos = node.pos
            
            node_index -= 1
        
        return total_cost
        


    def optimum_policy_2D(self, grid_cells, init_pos, goal_pos, turn_costs, cell_blockage_value):
        # self.value_table = np.ones(grid_cells.shape)
        # self.cost_table = np.ones_like(grid_cells) * RouteFinder.__cost_default_value__
        self.reset_cost_table(grid_cells.shape)
        
        self.cost_table[grid_cells == 1] = -1
        self.cost_table[goal_pos[0], goal_pos[1]] = 0
        
        # all_costs_are_estimated = False
        # prev_init_pos = []
        # init_pos = [-1, -1]
        
        depth_counter = 0
        
        # in case the route to goal is blocked 
        # max_fail_count = 10
        # pointer_counter = 0
        
        # while (True):
        navigable_indices = np.argwhere(self.cost_table == RouteFinder.__cost_default_value__)
                        
        # all_costs_are_estimated = remaining_indices.size == 0
        # print("remaining_indices", remaining_indices)
        # print("self.cost_table", self.cost_table)
                        
        # if (all_costs_are_estimated or prev_init_pos == init_pos or depth_counter == 10):
            
        if (all_costs_are_estimated or prev_init_pos == init_pos):
            self.cost_table[self.cost_table == -1] = cell_blockage_value
            # break
                        
        depth_counter += 1
        
        # if (depth_counter > 10):
        #    print("debug counter limit is reached")
        #    break
        
        copy_grid_cells = np.copy(grid_cells)
        # prev_init_pos = init_pos
        # init_pos = [remaining_indices[0][0], remaining_indices[0][1]]
            
        search_results = self.search_dynamic_programming(copy_grid_cells, init_pos, goal_pos, max_depth = 100, 
                                                         cell_blockage_value = cell_blockage_value, turn_costs = turn_costs)
        
        
        
        
        
        
        
        
        # print(self.cost_table)
        
        cost_table_indices = np.argwhere(np.logical_and(self.cost_table != cell_blockage_value, self.cost_table != RouteFinder.__cost_default_value__)).tolist()
        cost_table_indices.remove(goal_pos)
                
        neighbor_indices = [[], [], [], []]
        
        unique_index_id = goal_pos[0] * grid_cells.shape[1] + goal_pos[1]
        self.neighbors_indices_table[unique_index_id] = [[], [], [], []]
        delta_names = ["^", "v", "<", ">"]
        
        self.direction_table[goal_pos[0], goal_pos[1]] = "*"
        
        # print(cost_table_indices)
        
        for pos in cost_table_indices:
            unique_index_id = pos[0] * grid_cells.shape[1] + pos[1]
            
            # print(pos, unique_index_id)
            # (up_neighbor_indices, down_neighbor_indices, left_neighbor_indices, right_neighbor_indices) = route_finder.extract_neighbors_indices(unique_index_id)
            (neighbor_indices[0], neighbor_indices[1], neighbor_indices[2], neighbor_indices[3]) = self.extract_neighbors_indices(unique_index_id)
            costs = [self.cost_table[neighbor_pos[0], neighbor_pos[1]] if neighbor_pos != [] else 1.0e50 for neighbor_pos in neighbor_indices]
            
            index_min = np.argmin(costs)
            # print(pos, costs, index_min)
            self.direction_table[pos[0], pos[1]] = delta_names[index_min]






grid_cells = np.copy(grid_cells_07)

print("========== Basic Search: start ==========")
print("grid Cell:\n", grid_cells)
search_results = route_finder.search("basic", grid_cells, heuristic_cells, init_pos, original_goal_pos, max_depth = 100)
print(search_results)
print("count_table basic:\n", route_finder.count_table)
print("directions basic:\n", route_finder.direction_table)
print("cost_table basic:\n", route_finder.cost_table)
print("==================== End ====================")

print("")


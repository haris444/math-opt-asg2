def solve_restricted_master_problem(current_routes, customers_list, alpha_penalty, model_name="RMP"):
    """
    Solves the LP relaxation of the Restricted Master Problem.

    Args:
        current_routes (list of dicts): Each dict represents a route and should contain:
                                         'id': A unique route identifier.
                                         'cost': The travel distance of the route.
                                         'customers_served': A list of customer IDs served by this route.
        customers_list (list): List of all customer IDs (e.g., [1, 2, ..., N]).
        alpha_penalty (float): Penalty for using a vehicle/route.
        model_name (str): Name for the Gurobi model.

    Returns:
        tuple: (model, dual_prices)
               dual_prices is a dict {customer_id: dual_value}
               Returns (None, None) if solving fails.
    """
    rmp = gp.Model(model_name)
    rmp.setParam('OutputFlag', 0) # Suppress Gurobi output for RMP

    route_vars = {} # To store lambda_r variables

    # Define variables and objective
    obj_expr = gp.LinExpr()
    for idx, route_details in enumerate(current_routes):
        route_id = route_details.get('id', f"route_{idx}")
        # Cost C_r = (travel distance of route r) + alpha
        cost_r = route_details['cost'] + alpha_penalty
        
        route_vars[route_id] = rmp.addVar(obj=cost_r, vtype=GRB.CONTINUOUS, lb=0, name=f"lambda_{route_id}") # LP relaxation
        # For final IP solve, vtype=GRB.BINARY, lb=0, ub=1

    rmp.modelSense = GRB.MINIMIZE

    # Define constraints (Set Covering: sum(a_ir * lambda_r) >= 1 for each customer i)
    # Or Set Partitioning: sum(a_ir * lambda_r) == 1
    customer_constraints = {}
    for i in customers_list:
        # a_ir is 1 if customer i is in route_details['customers_served']
        expr = gp.quicksum(route_vars[r_id] for r_idx, r_details in enumerate(current_routes)
                           if i in r_details['customers_served']
                           for r_id in [r_details.get('id', f"route_{r_idx}")])
        customer_constraints[i] = rmp.addConstr(expr >= 1, name=f"cover_cust_{i}")

    rmp.optimize()

    if rmp.status == GRB.OPTIMAL:
        dual_prices = {i: customer_constraints[i].Pi for i in customers_list}
        # Store solution values if needed for debugging or initial IP solution
        # solution_routes = {r_id: var.X for r_id, var in route_vars.items() if var.X > 1e-6}
        return rmp, dual_prices
    else:
        print(f"RMP optimization failed with status: {rmp.status}")
        return None, None

def solve_master_ip(final_routes, customers_list, alpha_penalty, model_name="MasterIP"):
    """
    Solves the Master Problem as an Integer Program with the final set of routes.
    """
    master_ip = gp.Model(model_name)
    # master_ip.setParam('OutputFlag', 0) # Optional: show Gurobi output

    route_vars = {}
    obj_expr = gp.LinExpr()

    for idx, route_details in enumerate(final_routes):
        route_id = route_details.get('id', f"route_{idx}")
        cost_r = route_details['cost'] + alpha_penalty
        route_vars[route_id] = master_ip.addVar(obj=cost_r, vtype=GRB.BINARY, name=f"lambda_{route_id}")

    master_ip.modelSense = GRB.MINIMIZE

    customer_constraints = {}
    for i in customers_list:
        expr = gp.quicksum(route_vars[r_id] for r_idx, r_details in enumerate(final_routes)
                           if i in r_details['customers_served']
                           for r_id in [r_details.get('id', f"route_{r_idx}")])
        customer_constraints[i] = master_ip.addConstr(expr >= 1, name=f"cover_cust_{i}") # Or == 1 for partitioning

    master_ip.optimize()
    
    # Extract solution details (which routes are selected, objective value, etc.)
    if master_ip.status == GRB.OPTIMAL or (master_ip.status in [GRB.TIME_LIMIT, GRB.INTERRUPTED, GRB.SUBOPTIMAL] and master_ip.SolCount > 0) :
        print(f"Master IP Objective: {master_ip.ObjVal}")
        selected_routes_details = []
        for idx, route_details in enumerate(final_routes):
            route_id = route_details.get('id', f"route_{idx}")
            if route_vars[route_id].X > 0.5:
                selected_routes_details.append(route_details)
        print(f"Number of vehicles/routes used: {len(selected_routes_details)}")
        return master_ip, selected_routes_details
    else:
        print("Master IP did not solve to optimality or find a solution.")
        return master_ip, None
        
        
        
def column_generation_solver(initial_routes, customers_list, demands, service_times, 
                             time_windows_data, coordinates_data, vehicle_capacity, 
                             speed, alpha_penalty, max_cg_iterations=50):
    
    current_routes_pool = list(initial_routes) # Start with some initial feasible routes
    
    for iteration in range(max_cg_iterations):
        print(f"\n--- Column Generation Iteration: {iteration + 1} ---")
        print(f"Current number of routes in RMP: {len(current_routes_pool)}")

        # 1. Solve the Restricted Master Problem (LP Relaxation)
        rmp_model, dual_prices = solve_restricted_master_problem(current_routes_pool, 
                                                                 customers_list, 
                                                                 alpha_penalty)
        if rmp_model is None or dual_prices is None:
            print("Failed to solve RMP. Stopping.")
            break
        
        current_rmp_obj = rmp_model.ObjVal
        print(f"RMP Objective Value: {current_rmp_obj:.2f}")

        # 2. Solve the Pricing Subproblem (ESPPRC)
        # This is the complex part you'd implement based on Hoogeboom et al.
        # It needs all problem data (demands, service_times, time_windows_data, coordinates_data,
        # vehicle_capacity, speed) and the dual_prices from the RMP.
        
        # new_route_found is a boolean
        # best_new_route is a dict {'cost': travel_distance, 'customers_served': [...], 'id': ...}
        # reduced_cost is C_r - sum(pi_i * a_ir)
        
        # Conceptual call to your ESPPRC solver:
        best_new_route, reduced_cost = solve_espprc_with_multiple_time_windows(
            customers_list, demands, service_times, time_windows_data,
            coordinates_data, vehicle_capacity, speed, alpha_penalty,
            dual_prices
            # You'll also need depot information here
        )

        if best_new_route and reduced_cost < -1e-6: # Threshold for negative reduced cost
            print(f"Found new route with reduced cost: {reduced_cost:.2f}")
            # Ensure new route is not already (effectively) in the pool to avoid duplicates
            # This check might need to be more sophisticated (e.g. comparing customer sets)
            is_new = True
            for r_in_pool in current_routes_pool:
                if set(r_in_pool['customers_served']) == set(best_new_route['customers_served']) and \
                   abs(r_in_pool['cost'] - best_new_route['cost']) < 1e-6 : # Example similarity check
                    is_new = False
                    print("New route is similar to an existing one in the pool.")
                    break
            if is_new:
                 current_routes_pool.append(best_new_route)
            else: # No genuinely new route that improves things significantly
                print("No new improving route found by ESPPRC or it's a duplicate. Stopping CG.")
                break

        else:
            print("No new route with negative reduced cost found. LP optimal for current columns.")
            break # Exit loop if pricing problem proves optimality for the LP

    print("\n--- Column Generation Finished ---")
    print(f"Final number of routes generated: {len(current_routes_pool)}")

    # 3. Solve the Master Problem as an Integer Program with all generated columns
    print("\nSolving final Master Problem as IP...")
    final_ip_model, selected_routes = solve_master_ip(current_routes_pool, 
                                                      customers_list, 
                                                      alpha_penalty)
    
    if selected_routes:
        # Calculate final objective based on your actual cost definition
        final_total_travel_distance = sum(r['cost'] for r in selected_routes)
        final_num_vehicles = len(selected_routes)
        final_objective = final_total_travel_distance + alpha_penalty * final_num_vehicles
        
        print(f"\nFinal Solution from Master IP:")
        print(f"  Objective (IP): {final_ip_model.ObjVal:.2f} (This includes alpha in C_r)")
        print(f"  Calculated objective (dist + alpha*veh): {final_objective:.2f}")
        print(f"  Number of vehicles: {final_num_vehicles}")
        print(f"  Total travel distance: {final_total_travel_distance:.2f}")
        # for r_detail in selected_routes:
        #     print(f"  Route {r_detail.get('id', 'N/A')}: Customers {r_detail['customers_served']}, Cost {r_detail['cost']:.2f}")
        return final_ip_model, selected_routes, current_routes_pool
    else:
        print("No final integer solution found for the master problem.")
        return None, None, current_routes_pool
        
        
        
        
        
        
def solve_espprc_with_multiple_time_windows(
        customers_list, demands, service_times, time_windows_data,
        coordinates_data, vehicle_capacity, speed, alpha_penalty,
        dual_prices, depot_id=0, depot_max_time=float('inf')): # Add other necessary params
    """
    CONCEPTUAL placeholder for the Elementary Shortest Path Problem with Resource Constraints
    and Multiple Time Windows.
    
    This function needs to implement a sophisticated labeling algorithm as
    described in Hoogeboom et al. (2019).

    Args:
        customers_list: List of customer IDs.
        demands: Dict of customer demands.
        service_times: Dict of customer service times.
        time_windows_data: Dict of customer multiple time windows.
        coordinates_data: Dict of all location coordinates.
        vehicle_capacity: Vehicle capacity.
        speed: Vehicle speed.
        alpha_penalty: Penalty for using a vehicle.
        dual_prices: Dict of dual prices for each customer from RMP.
        depot_id: ID of the depot.
        depot_max_time: Planning horizon / max return time to depot.


    Returns:
        tuple: (best_new_route_dict, reduced_cost_float)
               best_new_route_dict: {'cost': travel_dist, 'customers_served': [...], 'id': ...} or None
               reduced_cost_float: The reduced cost of the best route, or float('inf')
    """
    print("    Running Pricing Subproblem (ESPPRC)...")
    # --- THIS IS WHERE THE COMPLEX LABELING ALGORITHM FROM THE PAPER GOES ---
    # 1. Initialize labels at the depot.
    #    A label typically contains: (current_node, visited_nodes_mask_or_set, current_load,
    #                                current_time_intervals_list, current_path_cost_for_intervals,
    #                                accumulated_dual_value_sum)
    #    The "cost" here is travel distance. Time intervals are for feasibility.

    # 2. Iteratively extend labels:
    #    For each non-dominated label L ending at node u:
    #        For each reachable, unvisited neighbor v:
    #            Create new label L_new by extending L to v.
    #            - Check capacity.
    #            - Calculate new time intervals at v based on L's intervals, service_time[u],
    #              travel_time(u,v), and v's multiple time windows. This is the core of
    #              Hoogeboom's "start time intervals"[cite: 118, 123, 133].
    #            - Update path travel distance.
    #            - Update accumulated duals.
    #            - If L_new is feasible and not dominated by existing labels at v, add it.

    # 3. Apply dominance rules (Proposition 3.1, 3.2 from paper [cite: 167, 204])
    #    This involves the phi(L1, L2) calculation (Algorithm 3.1 [cite: 173]).

    # 4. When a label reaches the depot (completing a route):
    #    - Calculate its actual cost C_r = total_travel_distance_of_route + alpha_penalty.
    #    - Calculate its reduced cost = C_r - accumulated_dual_value_sum.
    #    - Keep track of the route with the most negative reduced cost.

    # 5. Consider bidirectional search [cite: 36, 141] and heuristic pricing [cite: 36, 220]
    #    for efficiency.

    # --- Placeholder: In a real implementation, this would return actual results ---
    print("    ESPPRC Labeling Algorithm (from Hoogeboom et al.) needs to be implemented here.")
    
    # For now, let's simulate finding no new route to allow the loop to terminate.
    # In a real scenario, you would build routes and calculate their reduced costs.
    # If you had a simple heuristic route generator, you could call it here.
    
    # Example of what a found route might look like (if your ESPPRC found one):
    # found_route = {
    #     'id': f"esp_route_{some_unique_id}",
    #     'cost': 150.7, # This is purely travel distance
    #     'customers_served': [cust_a, cust_b, cust_c],
    #     # Other details like actual start/end times, chosen TWs per customer, etc.
    # }
    # C_r = found_route['cost'] + alpha_penalty
    # sum_duals = sum(dual_prices.get(c,0) for c in found_route['customers_served'])
    # current_reduced_cost = C_r - sum_duals
    # if current_reduced_cost < most_negative_reduced_cost_so_far:
    #    best_new_route = found_route
    #    reduced_cost = current_reduced_cost

    return None, float('inf') # Placeholder
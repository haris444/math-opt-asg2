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
    env = gp.Env(params=params)
    master_ip = gp.Model(model_name, env = env)
    master_ip.setParam('OutputFlag', 0) # Optional: show Gurobi output

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
    dual_prices, depot_id=0, depot_max_time=float('inf')):
    """
    Main entry point for solving ESPPRC with bidirectional search.
    """
    # Prepare data in expected format
    data = {
        'customers_list': customers_list,
        'demands': demands,
        'service_times': service_times,
        'time_windows_data': time_windows_data,
        'coordinates_data': coordinates_data,
        'vehicle_capacity': vehicle_capacity,
        'speed': speed,
        'depot_id': depot_id,
        'depot_info': {
            'id': depot_id,
            'time_window': (0, depot_max_time),
            'demand': 0,
            'service_time': 0
        },
        'dual_prices': dual_prices,
        'travel_times': {},  # Will be populated from existing data
        'travel_distances': {}  # Will be populated from existing data
    }

    # Populate travel times and distances if not already done
    # Assuming you have calculate_distance_matrix and calculate_travel_time_matrix functions
    if 'travel_distances' not in data or not data['travel_distances']:
        from your_utils import calculate_distance_matrix, calculate_travel_time_matrix
        data['travel_distances'] = calculate_distance_matrix(coordinates_data)
        data['travel_times'] = calculate_travel_time_matrix(data['travel_distances'], speed)

    # Call bidirectional ESPPRC
    routes = solve_espprc_bidirectional(data, dual_prices, alpha_penalty)

    if routes:
        # Return best route
        best_route = routes[0]  # Already sorted by reduced cost
        return best_route, best_route['reduced_cost']
    else:
        return None, float('inf')
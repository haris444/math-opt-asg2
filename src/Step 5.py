import math
# Assuming Label class, initialize_depot_label, extend_label_to_new_node,
# _calculate_tau_ij_from_paper are defined as in previous steps.

def _calculate_phi_L1_L2(label1: Label, label2: Label, data: dict) -> float:
    """
    Calculates phi(L_f^1, L_f^2) as per Algorithm 3.1 in the paper[cite: 345].
    This version adapts for a travel distance objective. The term
    max{x - L_L_f1(y), 0} in the paper's phi calculation (line 6 of Alg 3.1)
    relates to additional waiting time if L1 finishes service earlier at L_L_f1(y)
    but has to wait until x. For a pure travel distance objective, this waiting
    does not add to the distance cost. So, that term effectively becomes 0.

    Thus, phi becomes max(D_L1(y_for_x) - D_L2(z_for_x)), where D is accumulated distance.
    Algorithm 3.1 structure is to iterate over intervals of L2 (z) and L1 (y).
    """
    phi_val = -float('inf')
    epsilon = 1e-6 # A small value as used in paper's context for strict inequalities

    # dominant_forward_start_intervals are (E, L, D)
    intervals_L1 = label1.dominant_forward_start_intervals
    intervals_L2 = label2.dominant_forward_start_intervals

    if not intervals_L1 or not intervals_L2: # Should not happen if labels are valid
        return -float('inf')

    idx_y = 0
    for E_L2_z, L_L2_z, D_L2_z in intervals_L2:
        # Find the interval y in L1 that is active for departures x in [E_L2_z, L_L2_z]
        # This requires careful handling of how delta_L1(x) is determined.
        # Algorithm 3.1 compares an interval z of L2 with inter-interval gaps of L1
        # or overlaps with intervals of L1.

        # Iterate through L1's intervals/gaps relevant to L2's current interval z
        current_L1_idx = 0
        temp_max_for_z = -float('inf')

        # This loop implements the logic from Algorithm 3.1 lines 3-11 [cite: 345]
        # It iterates through y in F_L1 (intervals_L1)
        # For each z in F_L2 (current L2 interval)
        temp_y_idx = 0
        while temp_y_idx < len(intervals_L1):
            E_L1_y, L_L1_y, D_L1_y = intervals_L1[temp_y_idx]
            
            # Next interval start for L1, or infinity if y is the last
            E_L1_y_plus_1 = intervals_L1[temp_y_idx+1][0] if temp_y_idx + 1 < len(intervals_L1) else float('inf')

            # Check overlap: [E_L2_z, L_L2_z] with [E_L1_y, E_L1_y_plus_1[
            # This is the "departure time x" range considered in Alg 3.1
            overlap_start = max(E_L2_z, E_L1_y)
            overlap_end = min(L_L2_z, E_L1_y_plus_1 - epsilon)

            if overlap_start <= overlap_end: # If there's a relevant segment to check
                # x is chosen as the end of this overlapping segment (line 5 of Alg 3.1)
                # x = min(L_L2_z, E_L1_y_plus_1 - epsilon)
                # For this x, delta_L1(x) = D_L1_y + max(x - L_L1_y, 0)
                # For distance, max(x - L_L1_y, 0) is 0. So delta_L1(x) = D_L1_y
                # delta_L2(x) = D_L2_z (since x is within L2's interval z by definition of loop)
                current_phi_contribution = D_L1_y - D_L2_z
                if current_phi_contribution > temp_max_for_z:
                    temp_max_for_z = current_phi_contribution
            
            if L_L2_z < E_L1_y_plus_1 - epsilon : # Line 9 of Alg 3.1 condition
                break # Go to next start time interval of label L2 (outer loop)
            
            temp_y_idx += 1
        
        if temp_max_for_z > -float('inf'):
             if temp_max_for_z > phi_val:
                phi_val = temp_max_for_z
        # Handle case where L2 interval extends beyond all L1 intervals/gaps
        elif temp_y_idx == len(intervals_L1) and E_L2_z <= intervals_L1[-1][1]: # L2_z overlaps with last L1 interval
            E_L1_y_last, L_L1_y_last, D_L1_y_last = intervals_L1[-1]
            # Check overlap with [E_L1_y_last, L_L1_y_last]
            overlap_start = max(E_L2_z, E_L1_y_last)
            overlap_end = min(L_L2_z, L_L1_y_last)
            if overlap_start <= overlap_end:
                 current_phi_contribution = D_L1_y_last - D_L2_z
                 if current_phi_contribution > phi_val: # update overall phi_val directly
                    phi_val = current_phi_contribution


    return phi_val if phi_val > -float('inf') else 0.0 # If no valid comparison, phi might be 0 or some default


def check_dominance(label1: Label, label2: Label, data: dict) -> bool:
    """
    Checks if label1 dominates label2 based on Proposition 3.1[cite: 339, 88].
    Adapted for travel distance objective.

    Args:
        label1: The potentially dominating label.
        label2: The potentially dominated label.
        data: Preprocessed data.

    Returns:
        True if label1 dominates label2, False otherwise.
    """
    # Condition 1: v(L_f^1) = v(L_f^2)
    # This is implicitly true as this function will be called for labels ending at the same node.
    if label1.current_node_id != label2.current_node_id:
        return False # Should not happen if used correctly

    # Condition 2: S(L_f^1) subseteq S_bar(L_f^2) [cite: 339]
    # Using simplified S(L_f^1) subseteq S(L_f^2) for elementarity.
    # If L1 visits a customer not in L2, L1 cannot dominate L2 if L2 could still visit it.
    # For L1 to dominate L2, L1 must be "more general" or equally general.
    # This means L1 should not have visited *more* specific customers than L2.
    # So, label1.visited_customers_set must be a subset of or equal to label2.visited_customers_set
    if not label1.visited_customers_set.issubset(label2.visited_customers_set):
        return False

    # Condition 3: q(L_f^1) <= q(L_f^2) [cite: 339]
    if label1.current_load > label2.current_load:
        return False

    # Condition 4: E_L_f1(1) <= E_L_f2(1) [cite: 340]
    if label1.get_earliest_start_of_first_interval() > label2.get_earliest_start_of_first_interval():
        return False

    # Condition 5: phi(L_f^1, L_f^2) <= pi(L_f^1) - pi(L_f^2) [cite: 340]
    # where pi(L_f) is the sum of duals for label L_f.
    # The RHS is label1.accumulated_dual_value_sum - label2.accumulated_dual_value_sum
    # The paper's proof appendix C.2 uses c(L_f^1) - c(L_f^2) where c(L_f) is sum of duals[cite: 88].
    
    # If all previous conditions hold and L1 is strictly better in one resource
    # or has strictly lower dual sum for same resources, phi might not be needed.
    # But the paper's BCP includes it.
    
    # For distance, the accumulated_dual_value_sum is the "sum of duals".
    # The "cost" of a path in the objective is distance.
    # The condition 5 is about reduced cost comparison.
    # Reduced cost of L1 + extension <= Reduced cost of L2 + extension
    # (Dist1 + phi) - DualSum1 <= Dist2 - DualSum2 (if phi represents cost difference)
    # Dist1 - DualSum1 <= Dist2 - (DualSum2 + phi)
    # The paper's condition (Prop 3.1, page 47): phi(L1,L2) <= pi(L1) - pi(L2)
    # where pi(L) is sum of dual variables.
    
    # Calculate phi(label1, label2)
    # This phi should represent max(cost_L1_extension - cost_L2_extension) for any common extension.
    # If cost is just distance, and delta_Lf(x) = D_Lf(y), then phi simplifies.
    # Let's use the _calculate_phi_L1_L2 implementation.
    phi_1_2 = _calculate_phi_L1_L2(label1, label2, data)

    if phi_1_2 > (label1.accumulated_dual_value_sum - label2.accumulated_dual_value_sum) + 1e-6: # Add tolerance for float comparison
        return False

    # At least one of the conditions (load, earliest_start_time, or effective cost from Cond 5)
    # must be strictly better if the visited sets are identical. Or if L1 is a strict subset for visited.
    if label1.visited_customers_set == label2.visited_customers_set:
        if not (label1.current_load < label2.current_load - 1e-6 or
                label1.get_earliest_start_of_first_interval() < label2.get_earliest_start_of_first_interval() - 1e-6 or
                (phi_1_2 < (label1.accumulated_dual_value_sum - label2.accumulated_dual_value_sum) - 1e-6) ):
            # If all are equal or only phi makes them equal, it might not be a strict dominance
            # However, the paper does not require strict inequality for dominance.
            # Let's assume non-strict as per paper for now.
            pass


    return True


def solve_espprc_forward_labeling(
    data: dict,
    dual_prices: dict,
    alpha_penalty: float,
    depot_return_time_limit: float
):
    """
    Solves the Elementary Shortest Path Problem with Resource Constraints (ESPPRC)
    using a monodirectional forward labeling algorithm.

    Args:
        data (dict): Preprocessed problem data.
        dual_prices (dict): {customer_id: dual_value} from RMP.
        alpha_penalty (float): Cost for using a vehicle.
        depot_return_time_limit (float): Latest allowed arrival time back at the depot.


    Returns:
        list: A list of new routes (dicts) with negative reduced costs.
              Each route dict: {'path': list_of_nodes, 'cost': travel_distance,
                                'customers_served': list_of_customers,
                                'reduced_cost': float, 'id': str}
    """
    depot_id = data['depot_info']['id']
    customers_list = data['customers_list']
    vehicle_capacity = data['vehicle_capacity']

    initial_label = initialize_depot_label(data)
    
    # Stores non-dominated labels found so far for each node
    # key: node_id, value: list of Label objects
    non_dominated_labels_at_node = {node_id: [] for node_id in [depot_id] + customers_list}
    non_dominated_labels_at_node[depot_id].append(initial_label)

    # Labels to be processed
    unprocessed_labels = [initial_label]

    generated_negative_routes = []
    min_reduced_cost_overall = 0.0 # Track the most negative reduced cost

    iteration_count = 0 # Safety break for very long runs

    while unprocessed_labels:
        iteration_count += 1
        # if iteration_count % 100 == 0:
        #     print(f"  ESPPRC Iteration: {iteration_count}, Unprocessed: {len(unprocessed_labels)}, Neg Routes: {len(generated_negative_routes)}")
        # if iteration_count > 20000: # Safety break
        #     print("  ESPPRC safety break: Max iterations reached.")
        #     break

        current_L = unprocessed_labels.pop(0) # FIFO processing for now

        # Try to extend to other customers
        for next_cust_id in customers_list:
            if next_cust_id not in current_L.visited_customers_set:
                # 1. Check capacity
                if current_L.current_load + data['demands'].get(next_cust_id, 0) <= vehicle_capacity:
                    # 2. Generate dominant forward start intervals at next_cust_id
                    # extend_label_to_new_node was the previous name
                    new_intervals_at_next_node = extend_label_to_new_node(current_L, next_cust_id, data)

                    if new_intervals_at_next_node: # Feasible extension found
                        new_path_seq = current_L.path_sequence + [next_cust_id]
                        new_visited_set = current_L.visited_customers_set.union({next_cust_id})
                        new_load = current_L.current_load + data['demands'].get(next_cust_id, 0)
                        new_dual_sum = current_L.accumulated_dual_value_sum + dual_prices.get(next_cust_id, 0.0)

                        new_label = Label(
                            current_node_id=next_cust_id,
                            visited_customers_set=new_visited_set,
                            current_load=new_load,
                            dominant_forward_start_intervals=new_intervals_at_next_node,
                            accumulated_dual_value_sum=new_dual_sum,
                            path_sequence=new_path_seq
                        )

                        # Dominance Check (Proposition 3.1)
                        is_dominated_by_existing = False
                        kept_labels_for_next_node = []
                        for existing_label in non_dominated_labels_at_node[next_cust_id]:
                            if check_dominance(existing_label, new_label, data):
                                is_dominated_by_existing = True
                                break
                            if not check_dominance(new_label, existing_label, data):
                                kept_labels_for_next_node.append(existing_label)
                        
                        if not is_dominated_by_existing:
                            non_dominated_labels_at_node[next_cust_id] = kept_labels_for_next_node + [new_label]
                            unprocessed_labels.append(new_label)
                            # Sort unprocessed_labels? e.g. by path length or dual sum for heuristic processing order
                            # unprocessed_labels.sort(key=lambda l: len(l.path_sequence))


        # Try to extend current_L (ending at current_L.current_node_id) back to the depot
        if current_L.current_node_id != depot_id : # Path has at least one customer
            path_ending_node_id = current_L.current_node_id
            
            # tau_ij for returning to depot: service_at_path_ending_node + travel_time_to_depot
            tau_to_depot = _calculate_tau_ij_from_paper(path_ending_node_id, depot_id, data)
            dist_to_depot = data['travel_distances'].get((path_ending_node_id, depot_id), float('inf'))

            min_route_distance_for_this_path = float('inf')
            feasible_return = False

            for E_L_y, L_L_y, D_L_y in current_L.dominant_forward_start_intervals:
                # Earliest completion time at path_ending_node_id if service started at E_L_y
                # Service starts at E_L_y, duration is service_time. Travel starts after service.
                # Arrival at depot: E_L_y (start service) + tau_to_depot
                arrival_at_depot = E_L_y + tau_to_depot
                
                if arrival_at_depot <= depot_return_time_limit:
                    current_total_dist = D_L_y + dist_to_depot
                    if current_total_dist < min_route_distance_for_this_path:
                        min_route_distance_for_this_path = current_total_dist
                    feasible_return = True
            
            if feasible_return:
                # Route cost = total_travel_distance + alpha_penalty
                route_cost_C_r = min_route_distance_for_this_path + alpha_penalty
                # Reduced cost = C_r - sum(dual_prices for customers in path)
                # current_L.accumulated_dual_value_sum already has sum for customers in path_sequence[1:]
                sum_duals_for_path = current_L.accumulated_dual_value_sum
                
                reduced_cost = route_cost_C_r - sum_duals_for_path

                if reduced_cost < 1e-6 : # Threshold for negativity
                    # print(f"  Found path {current_L.path_sequence + [depot_id]} to depot. Dist: {min_route_distance_for_this_path:.2f}, RC: {reduced_cost:.2f}")
                    route_details = {
                        'path': current_L.path_sequence + [depot_id],
                        'cost': min_route_distance_for_this_path, # This is pure travel distance
                        'customers_served': list(current_L.visited_customers_set),
                        'reduced_cost': reduced_cost,
                        'id': f"gen_route_{len(generated_negative_routes)}_{iteration_count}"
                    }
                    generated_negative_routes.append(route_details)
                if reduced_cost < min_reduced_cost_overall:
                    min_reduced_cost_overall = reduced_cost

    # Sort by reduced cost to return the best ones first (most negative)
    generated_negative_routes.sort(key=lambda r: r['reduced_cost'])
    return generated_negative_routes
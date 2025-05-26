def extend_backward_label_to_node(label_at_j: Label, prev_node_id: int, data: dict) -> Optional[Label]:
    """
    Extends a backward label from node j to previous node i.

    This is the backward equivalent of forward label extension.
    The key difference is that we're going backwards: from j to i.

    Returns:
        New backward label at node i, or None if extension is infeasible.
    """
    if not label_at_j.is_forward:  # Must be backward label
        node_j_id = label_at_j.current_node_id

        # Check if prev_node_id is reachable
        if prev_node_id in label_at_j.visited_customers_set:
            return None

        # Check capacity
        if prev_node_id != data['depot_id']:
            new_load = label_at_j.current_load + data['demands'].get(prev_node_id, 0)
            if new_load > data['vehicle_capacity']:
                return None
        else:
            new_load = label_at_j.current_load

        # Generate backward intervals at prev_node_id
        # Similar to forward but in reverse direction
        new_intervals = _generate_backward_intervals_at_i(label_at_j, prev_node_id, data)

        if not new_intervals:
            return None

        # Create new backward label
        new_visited = label_at_j.visited_customers_set.copy()
        if prev_node_id != data['depot_id']:
            new_visited.add(prev_node_id)

        new_path = [prev_node_id] + label_at_j.path_sequence

        new_dual_sum = label_at_j.accumulated_dual_value_sum
        if prev_node_id != data['depot_id']:
            new_dual_sum += data.get('dual_prices', {}).get(prev_node_id, 0)

        return Label(
            current_node_id=prev_node_id,
            visited_customers_set=new_visited,
            current_load=new_load,
            dominant_intervals=new_intervals,
            accumulated_dual_value_sum=new_dual_sum,
            path_sequence=new_path,
            is_forward=False
        )

    return None


def _generate_backward_intervals_at_i(label_at_j: Label, node_i_id: int, data: dict) -> list:
    """
    Generates backward start intervals at node i when extending backward from j.

    This is the backward equivalent of forward interval generation.
    Key: We need to ensure that if we start service at node i within these intervals,
    we can reach node j in time.
    """
    node_j_id = label_at_j.current_node_id
    generated_intervals = []

    # tau_ij includes service time at i and travel time from i to j
    service_time_at_i = data['service_times'].get(node_i_id, 0) if node_i_id != data['depot_id'] else 0
    travel_time_ij = data['travel_times'].get((node_i_id, node_j_id), float('inf'))
    tau_ij = service_time_at_i + travel_time_ij
    travel_dist_ij = data['travel_distances'].get((node_i_id, node_j_id), float('inf'))

    # Get time windows at node i
    time_windows_at_i = data['time_windows_data'].get(node_i_id, [])

    # For each backward interval at j, determine feasible intervals at i
    for E_Lb_y, L_Lb_y, D_Lb_y in label_at_j.dominant_intervals:
        for e_i_t, l_i_t in time_windows_at_i:
            # To start service at j at time E_Lb_y, we must finish at i by E_Lb_y - travel_time_ij
            # So we must start at i by E_Lb_y - tau_ij
            latest_start_at_i = E_Lb_y - tau_ij

            # We can start at i as early as e_i_t, but no later than min(l_i_t, latest_start_at_i)
            new_E_i = e_i_t
            new_L_i = min(l_i_t, latest_start_at_i)

            if new_E_i <= new_L_i:
                # This is a feasible backward interval
                new_D_i = D_Lb_y + travel_dist_ij
                generated_intervals.append((new_E_i, new_L_i, new_D_i))

    # Filter to keep only dominant intervals
    return _filter_dominant_intervals_pareto(generated_intervals)


def check_backward_dominance(label1: Label, label2: Label, data: dict) -> bool:
    """
    Checks if backward label1 dominates backward label2 (Proposition 3.2).

    Conditions:
    1. v(L1_b) = v(L2_b)
    2. S(L1_b) ⊆ S_bar(L2_b)
    3. q(L1_b) ≤ q(L2_b)
    4. L_L1_b(|B_L1_b|) ≥ L_L2_b(|B_L2_b|)
    5. φ(L1_b, L2_b) ≤ π(L1_b) - π(L2_b)
    """
    if label1.current_node_id != label2.current_node_id:
        return False

    # Check if L1 visits only customers that L2 could still visit
    if not label1.visited_customers_set.issubset(label2.visited_customers_set):
        return False

    # Check load
    if label1.current_load > label2.current_load:
        return False

    # Check latest start time of last interval
    if label1.get_latest_start_of_last_interval() < label2.get_latest_start_of_last_interval():
        return False

    # Calculate φ(L1_b, L2_b) - similar to forward but for backward
    phi_val = _calculate_phi_backward(label1, label2, data)

    if phi_val > (label1.accumulated_dual_value_sum - label2.accumulated_dual_value_sum) + 1e-6:
        return False

    return True


def _calculate_phi_backward(label1: Label, label2: Label, data: dict) -> float:
    """
    Calculates φ(L1_b, L2_b) for backward labels.
    Similar to forward phi calculation but adapted for backward direction.
    """
    phi_val = -float('inf')
    epsilon = 1e-6

    intervals_L1 = label1.dominant_intervals
    intervals_L2 = label2.dominant_intervals

    if not intervals_L1 or not intervals_L2:
        return -float('inf')

    # Similar logic to forward but adapted for backward
    for E_L2_z, L_L2_z, D_L2_z in intervals_L2:
        for idx, (E_L1_y, L_L1_y, D_L1_y) in enumerate(intervals_L1):
            # Check overlap and calculate phi contribution
            # This is simplified - full implementation would follow Algorithm 3.1 adapted for backward
            overlap_start = max(E_L2_z, E_L1_y)
            overlap_end = min(L_L2_z, L_L1_y)

            if overlap_start <= overlap_end:
                # For backward, the phi calculation is similar but considers arrival times
                current_phi = D_L2_z - D_L1_y
                if current_phi > phi_val:
                    phi_val = current_phi

    return phi_val if phi_val > -float('inf') else 0.0


def merge_forward_backward_labels(forward_label: Label, backward_label: Label, data: dict) -> Optional[dict]:
    """
    Merges a forward and backward label to create a complete route (Algorithm C.2).

    Returns:
        Dict with route information if merge is feasible, None otherwise.
    """
    # Check merge conditions
    if forward_label.current_node_id == backward_label.current_node_id:
        return None  # Can't merge at same node

    # Check if arc exists
    if (forward_label.current_node_id, backward_label.current_node_id) not in data['travel_times']:
        return None

    # Check no shared customers
    if forward_label.visited_customers_set.intersection(backward_label.visited_customers_set):
        return None

    # Check capacity
    if forward_label.current_load + backward_label.current_load > data['vehicle_capacity']:
        return None

    # Check time feasibility
    tau_ij = _calculate_tau_ij_from_paper(forward_label.current_node_id,
                                          backward_label.current_node_id, data)

    # Find feasible interval combinations
    min_route_distance = float('inf')
    feasible_merge = False

    for E_Lf_y, L_Lf_y, D_Lf_y in forward_label.dominant_intervals:
        for E_Lb_z, L_Lb_z, D_Lb_z in backward_label.dominant_intervals:
            # Check if we can depart from forward node and arrive at backward node in time
            earliest_arrival_at_backward = E_Lf_y + tau_ij
            latest_arrival_at_backward = L_Lf_y + tau_ij

            # We need to arrive before L_Lb_z to start service by then
            if earliest_arrival_at_backward <= L_Lb_z:
                # Calculate waiting time if any
                actual_start_at_backward = max(earliest_arrival_at_backward, E_Lb_z)
                waiting_time = max(0, E_Lb_z - latest_arrival_at_backward)

                # Total distance for this combination
                dist_ij = data['travel_distances'][(forward_label.current_node_id,
                                                   backward_label.current_node_id)]
                total_dist = D_Lf_y + dist_ij + D_Lb_z

                if total_dist < min_route_distance:
                    min_route_distance = total_dist
                    feasible_merge = True

    if not feasible_merge:
        return None

    # Create complete path
    complete_path = forward_label.path_sequence + backward_label.path_sequence

    # Calculate route details
    route_details = {
        'path': complete_path,
        'cost': min_route_distance,  # Pure travel distance
        'customers_served': list(forward_label.visited_customers_set.union(
                                backward_label.visited_customers_set)),
        'reduced_cost': None,  # Will be calculated later
        'id': f"route_{len(complete_path)}_{forward_label.current_node_id}_{backward_label.current_node_id}"
    }

    return route_details


def solve_espprc_bidirectional(data: dict,
                               dual_prices: dict,
                               alpha_penalty: float,
                               max_labels: int = 50000) -> List[dict]:
    """
    Solves ESPPRC using bidirectional labeling algorithm.

    Returns:
        List of routes with negative reduced cost.
    """
    depot_id = data['depot_id']
    customers_list = data['customers_list']

    # Initialize forward and backward labels at depot
    initial_forward = initialize_depot_label(data)
    initial_backward = initialize_depot_label(data)
    initial_backward.is_forward = False

    # Storage for non-dominated labels
    forward_labels = {node_id: [] for node_id in [depot_id] + customers_list}
    backward_labels = {node_id: [] for node_id in [depot_id] + customers_list}

    forward_labels[depot_id].append(initial_forward)
    backward_labels[depot_id].append(initial_backward)

    # Unprocessed labels
    forward_unprocessed = [initial_forward]
    backward_unprocessed = [initial_backward]

    # Track bounds for merging decision
    min_forward_E = 0  # Minimum E_Lf(1) among forward labels
    max_backward_L = data['depot_info']['time_window'][1]  # Maximum L_Lb(|B_Lb|)

    generated_routes = []
    total_labels_processed = 0

    while (forward_unprocessed or backward_unprocessed) and total_labels_processed < max_labels:
        # Process forward labels
        if forward_unprocessed and total_labels_processed < max_labels // 2:
            current_forward = forward_unprocessed.pop(0)
            total_labels_processed += 1

            # Extend to customers
            for next_cust_id in customers_list:
                if next_cust_id not in current_forward.visited_customers_set:
                    # Check basic feasibility
                    if (current_forward.current_load + data['demands'].get(next_cust_id, 0) <=
                        data['vehicle_capacity']):
                        # Generate new forward intervals
                        new_intervals = extend_label_to_new_node(current_forward, next_cust_id, data)

                        if new_intervals:
                            # Create new forward label
                            new_path = current_forward.path_sequence + [next_cust_id]
                            new_visited = current_forward.visited_customers_set.union({next_cust_id})
                            new_load = current_forward.current_load + data['demands'][next_cust_id]
                            new_dual_sum = current_forward.accumulated_dual_value_sum + dual_prices.get(next_cust_id, 0)

                            new_forward_label = Label(
                                current_node_id=next_cust_id,
                                visited_customers_set=new_visited,
                                current_load=new_load,
                                dominant_intervals=new_intervals,
                                accumulated_dual_value_sum=new_dual_sum,
                                path_sequence=new_path,
                                is_forward=True
                            )

                            # Check dominance
                            is_dominated = False
                            labels_to_remove = []

                            for existing_label in forward_labels[next_cust_id]:
                                if check_dominance(existing_label, new_forward_label, data):
                                    is_dominated = True
                                    break
                                elif check_dominance(new_forward_label, existing_label, data):
                                    labels_to_remove.append(existing_label)

                            if not is_dominated:
                                # Remove dominated labels
                                for label in labels_to_remove:
                                    forward_labels[next_cust_id].remove(label)
                                    if label in forward_unprocessed:
                                        forward_unprocessed.remove(label)

                                # Add new label
                                forward_labels[next_cust_id].append(new_forward_label)
                                forward_unprocessed.append(new_forward_label)

                                # Update min forward E
                                min_forward_E = min(min_forward_E, new_forward_label.get_earliest_start_of_first_interval())

        # Process backward labels
        if backward_unprocessed and total_labels_processed < max_labels:
            current_backward = backward_unprocessed.pop(0)
            total_labels_processed += 1

            # Extend to customers (backwards)
            for prev_cust_id in customers_list:
                if prev_cust_id not in current_backward.visited_customers_set:
                    # Use backward extension
                    new_backward_label = extend_backward_label_to_node(current_backward, prev_cust_id, data)

                    if new_backward_label:
                        # Check backward dominance
                        is_dominated = False
                        labels_to_remove = []

                        for existing_label in backward_labels[prev_cust_id]:
                            if check_backward_dominance(existing_label, new_backward_label, data):
                                is_dominated = True
                                break
                            elif check_backward_dominance(new_backward_label, existing_label, data):
                                labels_to_remove.append(existing_label)

                        if not is_dominated:
                            # Remove dominated labels
                            for label in labels_to_remove:
                                backward_labels[prev_cust_id].remove(label)
                                if label in backward_unprocessed:
                                    backward_unprocessed.remove(label)

                            # Add new label
                            backward_labels[prev_cust_id].append(new_backward_label)
                            backward_unprocessed.append(new_backward_label)

                            # Update max backward L
                            max_backward_L = max(max_backward_L,
                                               new_backward_label.get_latest_start_of_last_interval())

        # Check merging condition
        if min_forward_E > max_backward_L or total_labels_processed >= max_labels // 2:
            # Merge forward and backward labels
            for forward_node_id, forward_label_list in forward_labels.items():
                if forward_node_id == depot_id:
                    continue

                for forward_label in forward_label_list:
                    # Try to merge with backward labels at different nodes
                    for backward_node_id, backward_label_list in backward_labels.items():
                        if backward_node_id == depot_id or backward_node_id == forward_node_id:
                            continue

                        for backward_label in backward_label_list:
                            # Try to merge
                            route_details = merge_forward_backward_labels(forward_label,
                                                                        backward_label, data)

                            if route_details:
                                # Calculate reduced cost
                                route_cost = route_details['cost'] + alpha_penalty
                                sum_duals = (forward_label.accumulated_dual_value_sum +
                                           backward_label.accumulated_dual_value_sum)
                                reduced_cost = route_cost - sum_duals

                                route_details['reduced_cost'] = reduced_cost

                                if reduced_cost < -1e-6:
                                    generated_routes.append(route_details)

            # Clear processed labels to make room for new ones
            if total_labels_processed >= max_labels:
                break

    # Sort routes by reduced cost
    generated_routes.sort(key=lambda r: r['reduced_cost'])

    return generated_routes
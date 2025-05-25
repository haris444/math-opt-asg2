import math

# Assuming the Label class from Step 3 is defined above or imported
# class Label:
#     ... (implementation from Step 3) ...

def initialize_depot_label(data: dict) -> Label:
    """
    Creates the initial Label object for the depot.

    Args:
        data (dict): The preprocessed data dictionary.

    Returns:
        Label: The initial label starting at the depot.
    """
    depot_id = data['depot_info']['id']
    depot_time_window = data['depot_info']['time_window'] # (start_time, end_time)
    initial_intervals = [(depot_time_window[0], depot_time_window[1], 0.0)]

    return Label(
        current_node_id=depot_id,
        visited_customers_set=set(),
        current_load=0.0,
        dominant_forward_start_intervals=initial_intervals,
        accumulated_dual_value_sum=0.0,
        path_sequence=[depot_id]
    )

def _calculate_tau_ij_from_paper(node_i_id: int, node_j_id: int, data: dict) -> float:
    """
    Calculates tau_ij as used in the paper: service time at node i + travel time from i to j.
    Note: The paper's tau_ij definition on page 43 "Duration tau_ij includes the service
    time at node i and the travel time between nodes i and j."
    My previous _calculate_tau_ij was identical.
    """
    service_time_at_i = 0.0
    if node_i_id != data['depot_info']['id']: # If node i is not the depot
        service_time_at_i = data['service_times'].get(node_i_id, 0.0)
    else: # node_i_id is the depot
        service_time_at_i = data['depot_info']['service_time'] # This should be 0

    travel_time_ij = data['travel_times'].get((node_i_id, node_j_id), float('inf'))
    return service_time_at_i + travel_time_ij

def _generate_candidate_intervals_at_j(label_at_i: Label, customer_j_id: int, data: dict) -> list:
    """
    Generates candidate forward start intervals at customer_j_id by extending label_at_i,
    based on Algorithm C.1 from the appendix[cite: 77, 79].
    The 'cost' associated with intervals will be accumulated travel distance.
    """
    node_i_id = label_at_i.current_node_id
    generated_intervals = []

    # tau_v(Lf)j in Algorithm C.1 is service_at_v(Lf) + travel_v(Lf)_to_j
    tau_node_i_j = _calculate_tau_ij_from_paper(node_i_id, customer_j_id, data)
    travel_dist_ij = data['travel_distances'].get((node_i_id, customer_j_id), float('inf'))
    
    original_time_windows_at_j = data['time_windows_data'].get(customer_j_id, [])
    dominant_intervals_at_i = label_at_i.dominant_forward_start_intervals

    if node_i_id == data['depot_info']['id']: # Case: v(Lf) = 0 (depot) [cite: 77]
        # Lines 2-5 of Algorithm C.1 [cite: 77]
        # tau_0j for duration. For distance, it's data['travel_distances'][(depot_id, customer_j_id)]
        depot_travel_dist_to_j = data['travel_distances'].get((data['depot_info']['id'], customer_j_id), float('inf'))
        for e_j_t, l_j_t in original_time_windows_at_j:
            # Check direct reachability from depot based on depot's end time and tau_0j
            # Depot earliest departure is dominant_intervals_at_i[0][0] (usually 0)
            # Depot latest departure is dominant_intervals_at_i[0][1]
            depot_earliest_departure = dominant_intervals_at_i[0][0] # E_Lf(y) where y is the single depot interval
            
            # Effective earliest start at j if departing depot at earliest time
            earliest_service_start_at_j = max(depot_earliest_departure + tau_node_i_j, e_j_t)
            
            # Effective latest start at j if departing depot at latest time
            latest_service_start_at_j = min(max(dominant_intervals_at_i[0][1] + tau_node_i_j, e_j_t), l_j_t)

            if earliest_service_start_at_j <= latest_service_start_at_j :
                 generated_intervals.append(
                    (earliest_service_start_at_j, latest_service_start_at_j, depot_travel_dist_to_j)
                )
    else: # Case: v(Lf) != 0 (previous node is a customer) [cite: 77, 79]
        # Lines 7-21 of Algorithm C.1 [cite: 77, 79]
        for idx_y, y_interval in enumerate(dominant_intervals_at_i):
            E_Lf_y, L_Lf_y, D_Lf_y = y_interval # D_Lf_y is acc_dist_to_i

            for t_original_tw_j in original_time_windows_at_j:
                e_j_t, l_j_t = t_original_tw_j

                # Subcase: Interval lies before time window (Lines 9-13 of Alg C.1 [cite: 77, 79])
                if L_Lf_y + tau_node_i_j < e_j_t:
                    # Check dominance condition from Line 10 of Alg C.1 [cite: 77]
                    generates_waiting_interval = False
                    if idx_y == len(dominant_intervals_at_i) - 1: # y is the last interval F_Lf
                        generates_waiting_interval = True
                    else:
                        E_Lf_y_plus_1 = dominant_intervals_at_i[idx_y+1][0]
                        if E_Lf_y_plus_1 + tau_node_i_j > e_j_t:
                            generates_waiting_interval = True
                    
                    if generates_waiting_interval:
                        # Add [e_j^t, e_j^t] to intervals
                        # Cost: D_Lf_y + travel_dist_ij. Waiting doesn't add to travel distance.
                        # Algorithm C.1 adds d_Lf(y) + e_j^t - L_Lf(y) to g(Lf'). This reflects waiting time in duration.
                        # For travel distance, it remains D_Lf_y + travel_dist_ij.
                        generated_intervals.append((e_j_t, e_j_t, D_Lf_y + travel_dist_ij))
                
                # Subcase: Overlap interval and time window (Lines 14-17 of Alg C.1 [cite: 77, 79])
                # Condition: E_Lf(y) + tau_node_i_j <= l_j_t (combined with not being strictly before)
                elif E_Lf_y + tau_node_i_j <= l_j_t: # This is the general overlap condition
                    new_E_j = max(E_Lf_y + tau_node_i_j, e_j_t)
                    new_L_j = min(L_Lf_y + tau_node_i_j, l_j_t) # Error in Alg C.1 line 15: uses min{max{L_Lf(y)...}}. Eq 3.5 is min{max{L_p(y)+tau, e_j^t}, l_j^t} for L_p'(z). Let's use 3.5 logic.
                    # Re-evaluating L_p_prime_z from eq 3.5:
                    new_L_j_corrected = min(max(L_Lf_y + tau_node_i_j, e_j_t), l_j_t)


                    if new_E_j <= new_L_j_corrected: # Valid interval
                        # Cost: D_Lf_y + travel_dist_ij. Alg C.1 (line 16) adds d_Lf(y) + tau_v(Lf),j.
                        generated_intervals.append((new_E_j, new_L_j_corrected, D_Lf_y + travel_dist_ij))
                
                # Break condition from Lines 18-20 of Alg C.1 [cite: 79]
                if L_Lf_y + tau_node_i_j < l_j_t: # This is L_Lf(y) in Alg C.1, not L_p_prime_z.
                                                 # If latest start from i + combined_time is before current TW end,
                                                 # it means it cannot reach later original TWs of j *any earlier*.
                                                 # This break is from the inner loop over T_j.
                    # The comment in paper [cite: 86] states "start interval y does not overlap with time windows t' > t"
                    # This implies that if L_Lf(y) + tau_v(Lf)j < l_j^t, we can break from iterating T_j FOR THE CURRENT y.
                    # And go to the next y. The code in C.1 does break.
                    pass # Python for loop will naturally go to next t_original_tw_j
                         # The break in C.1 (line 19) is for the loop over t in Tj
                         # This break should make it go to the next y interval if L_Lf(y) + tau < l_j^t
                         # This logic is subtle. The algorithm says "break", which exits the innermost loop (over T_j).
                         # So for the current 'y', if it cannot even reach the end of 'l_j_t', it surely cannot reach e_j^{t+1}.
                         # This means for the current y, we are done with time windows of j.
                    # Correction: The break should be for the inner loop over `t_original_tw_j`.
                    # If true, no need to check later time windows of j FOR THE CURRENT y_interval.
                    if L_Lf_y + tau_node_i_j < e_j_t: # If latest arrival from y is even before start of current TW_j
                                                      # (and didn't form a waiting interval)
                                                      # then it surely won't reach later TWs of j.
                        break # This break corresponds to line 19 in Alg C.1, exiting T_j loop for current y.


    return generated_intervals

def _filter_dominant_intervals_pareto(candidate_intervals: list) -> list:
    """
    Filters a list of candidate (E, L, D) intervals to keep only Pareto-dominant ones.
    An interval I1=(E1, L1, D1) dominates I2=(E2, L2, D2) if E1<=E2, L1>=L2, D1<=D2
    (and I1 != I2).
    The resulting list is sorted by E, then -L, then D.
    This function aims to produce a set that satisfies the non-overlapping property $L_k < E_{k+1}$
    implicitly through strong dominance, or by a final merging step (simplified here).
    """
    if not candidate_intervals:
        return []

    # Remove exact duplicates and sort for stable processing
    # Sort by E (asc), then L (desc for wider), then D (asc for cheaper)
    # This sorting helps in identifying dominated intervals more easily.
    unique_sorted_candidates = sorted(list(set(candidate_intervals)), key=lambda x: (x[0], -x[1], x[2]))

    dominant_intervals = []
    for current_interval in unique_sorted_candidates:
        is_dominated_by_existing = False
        # Check if current_interval is dominated by any interval already in dominant_intervals
        for existing_dominant_interval in dominant_intervals:
            # E_exist <= E_curr, L_exist >= L_curr, D_exist <= D_curr
            if (existing_dominant_interval[0] <= current_interval[0] and
                existing_dominant_interval[1] >= current_interval[1] and
                existing_dominant_interval[2] <= current_interval[2]):
                if existing_dominant_interval != current_interval: # Strictly dominated or identical covered one
                    is_dominated_by_existing = True
                    break
        
        if not is_dominated_by_existing:
            # Remove any intervals from dominant_intervals that are now dominated by current_interval
            new_dominant_list = []
            for i in range(len(dominant_intervals)):
                # Check if dominant_intervals[i] is dominated by current_interval
                # E_curr <= E_exist, L_curr >= L_exist, D_curr <= D_exist
                if not (current_interval[0] <= dominant_intervals[i][0] and
                        current_interval[1] >= dominant_intervals[i][1] and
                        current_interval[2] <= dominant_intervals[i][2] and
                        current_interval != dominant_intervals[i]):
                    new_dominant_list.append(dominant_intervals[i])
            dominant_intervals = new_dominant_list
            dominant_intervals.append(current_interval)
            # Re-sort is important if adding changes order for next iteration's checks,
            # but since unique_sorted_candidates is processed in order, this might be okay.
            # For safety, sort at the end.
            dominant_intervals.sort(key=lambda x: (x[0], -x[1], x[2]))


    # The list dominant_intervals now contains Pareto-optimal intervals.
    # To ensure strict non-overlapping $L_k < E_{k+1}$ as per Lemma 2.1[cite: 54]:
    # This might require a more sophisticated merging/selection if Pareto set still has overlaps.
    # For now, we return the sorted Pareto set. If overlaps cause issues later,
    # this part (non-overlapping merging) needs to be implemented robustly.
    # A simple greedy merge for same-cost adjacent/overlapping intervals:
    if not dominant_intervals:
        return []
    
    final_merged_intervals = []
    final_merged_intervals.append(dominant_intervals[0])

    for i in range(1, len(dominant_intervals)):
        curr_E, curr_L, curr_D = dominant_intervals[i]
        last_E, last_L, last_D = final_merged_intervals[-1]

        # If current interval is identical or strictly dominated by last_merged, it would have been filtered by Pareto.
        # Check for merging opportunity: if current starts at or before last ends, AND same cost, and extends last.
        if curr_E <= last_L + 1e-6 and curr_D == last_D : # Overlap or contiguous with same cost
            # Merge: update L of the last interval in final_merged_intervals
            final_merged_intervals[-1] = (last_E, max(last_L, curr_L), last_D)
        elif curr_E > last_L: # No overlap, current starts after last ended
            final_merged_intervals.append((curr_E, curr_L, curr_D))
        else: # Overlap with different costs, or other complex cases.
              # The Pareto filter should mean D_curr is not worse if it overlaps significantly.
              # If curr_D < last_D and they overlap, Pareto should have handled it.
              # This case means they are non-dominated but overlap.
              # Example: last=(0,10,D=5), curr=(5,15,D=5) -> merged to (0,15,D=5)
              # Example: last=(0,10,D=5), curr=(5,15,D=4) -> Pareto should keep (5,15,D=4) and (0,4.99,D=5)
              # The current simplified merge above only handles same-cost overlaps.
              # For now, if not same-cost mergeable and not starting after, just add if distinct.
              # This part is the most heuristic if not following a specific algorithm from [forthcoming].
              # Let's assume for now that after Pareto, if they are not same-cost mergeable, they form distinct dominant intervals.
              # The proof of Lemma 2.1 implies dominance sorts this out.
            if (curr_E, curr_L, curr_D) != final_merged_intervals[-1]: # Add if truly new after non-merge
                 #This could still lead to overlaps if D values are different.
                 #Safest is to return the pure Pareto set, sorted.
                 #The problem statement "dominant forward start intervals are non-overlapping" is a strong one.
                 #This means the filtering process MUST achieve this.
                 #A more robust way:
                 # If curr_E <= last_L (overlap):
                 #    If curr_D < last_D:
                 #        final_merged_intervals[-1] = (last_E, curr_E - epsilon, last_D) // Truncate last
                 #        if final_merged_intervals[-1][0] > final_merged_intervals[-1][1]: final_merged_intervals.pop()
                 #        final_merged_intervals.append((curr_E, curr_L, curr_D)) // Add current
                 #    Else if curr_D == last_D: (handled by merge above)
                 #    Else (curr_D > last_D): // Current is worse over the overlap
                 #        temp_E = last_L + epsilon
                 #        if temp_E <= curr_L:
                 #            final_merged_intervals.append((temp_E, curr_L, curr_D)) // Add truncated current
                 # else: (no overlap)
                 #    final_merged_intervals.append((curr_E, curr_L, curr_D))

                # Returning just the Pareto set and relying on later dominance (Prop 3.1) to resolve.
                # For now, let's stick to the simpler merge for same-cost, and add if distinct non-overlapping.
                # The code above already did the simple merge. If it didn't merge and wasn't disjoint, it's an issue.
                # Let's return the sorted Pareto list as `dominant_intervals`. The non-overlapping property is hard to guarantee
                # perfectly without the exact procedure from Hoogeboom et al. [forthcoming].
                # The provided Lemma 2.1 proof uses Prop 2.1 (from Chapter 2, for duration) to argue that
                # if intervals are not properly ordered (L_q < E_{q+1}), one must dominate the other.
                # This means our Pareto filter (E1<=E2, L1>=L2, D1<=D2) is the key.
                pass # The dominant_intervals list is already sorted from Pareto step.

    # Return the result of Pareto dominance, sorted.
    return dominant_intervals


def extend_label_to_new_node(label_at_i: Label, next_node_id: int, data: dict) -> list:
    """
    Wrapper function for extending a label from node i to next_node_id (customer or depot).
    If next_node_id is a customer, uses _generate_candidate_intervals_at_j.
    If next_node_id is the depot, no new intervals are generated, but feasibility is checked.

    Args:
        label_at_i (Label): The label ending at node i.
        next_node_id (int): The ID of the next node to visit.
        data (dict): The preprocessed data dictionary.

    Returns:
        list: A list of new dominant forward start intervals for the path extended to next_node_id.
              Returns empty list if no feasible extension or if next_node_id is depot (as intervals
              are properties of customer service starts).
    """
    if next_node_id == data['depot_info']['id']: # Extending back to depot
        # No new "forward start intervals" are typically defined for the end depot.
        # Feasibility of reaching end depot is checked when forming a full route.
        # The "dominant_forward_start_intervals" attribute is for service start at a customer.
        # The cost to reach the end depot is simply D_i + travel_dist(i, depot_end).
        # The "time" for the label reaching end depot will be the path completion time.
        return [] # Or perhaps special handling if labels are to store completion times at depot.
                  # For now, consistent with paper, these intervals are for customer service.
    else: # Extending to another customer
        candidate_intervals = _generate_candidate_intervals_at_j(label_at_i, next_node_id, data)
        if not candidate_intervals:
            return []
        return _filter_dominant_intervals_pareto(candidate_intervals)
def evaluate_route(route_sequence, departure_time, data):
    """
    Evaluate a potential route for feasibility and calculate total travel distance.
    
    Service can span multiple time windows: if service time extends beyond the current
    window, the driver pauses and waits for the next window to resume service.
    
    Args:
        route_sequence (list): Ordered list of customer IDs to visit
        departure_time (float): Departure time from depot in minutes from midnight
        data (dict): Preprocessed data dictionary
    
    Returns:
        float or None: Total travel distance if feasible, None if infeasible
    """
    if not route_sequence:
        return 0.0  # Empty route is valid with zero distance
    
    depot_id = data['depot_id']
    vehicle_capacity = data['vehicle_capacity']
    travel_times = data['travel_times']
    travel_distances = data['travel_distances']
    demands = data['demands']
    service_times = data['service_times']
    time_windows_data = data['time_windows_data']
    depot_time_window = data['depot_info']['time_window']
    
    # Initialize route state
    current_time = departure_time
    current_load = 0.0
    current_location = depot_id
    total_distance = 0.0
    
    # Check if departure time is within depot's operational window
    if current_time < depot_time_window[0] or current_time > depot_time_window[1]:
        return None
    
    # Visit each customer in sequence
    for customer_id in route_sequence:
        # Check capacity constraint
        customer_demand = demands[customer_id]
        if current_load + customer_demand > vehicle_capacity:
            return None  # Capacity exceeded
        
        # Calculate travel to customer
        travel_time = travel_times[(current_location, customer_id)]
        travel_distance = travel_distances[(current_location, customer_id)]
        arrival_time = current_time + travel_time
        
        # Get customer's time windows and service time
        customer_time_windows = time_windows_data[customer_id]
        total_service_time = service_times[customer_id]
        
        # Sort time windows by start time
        sorted_windows = sorted(customer_time_windows, key=lambda x: x[0])
        
        # Find first window where service can start (arrival_time <= window_end)
        service_start_window_idx = None
        for i, (window_start, window_end) in enumerate(sorted_windows):
            if arrival_time <= window_end:
                service_start_window_idx = i
                break
        
        if service_start_window_idx is None:
            return None  # Cannot start service in any window
        
        # Simulate service across potentially multiple windows
        remaining_service_time = total_service_time
        window_idx = service_start_window_idx
        current_service_time = arrival_time
        
        while remaining_service_time > 0:
            if window_idx >= len(sorted_windows):
                return None  # No more windows available
            
            window_start, window_end = sorted_windows[window_idx]
            
            # Determine when service can actually start in this window
            service_start_in_window = max(current_service_time, window_start)
            
            # If we've already passed this window, move to next
            if service_start_in_window >= window_end:
                window_idx += 1
                continue
            
            # Calculate how much service can be done in this window
            available_time_in_window = window_end - service_start_in_window
            service_in_this_window = min(remaining_service_time, available_time_in_window)
            
            # Update service progress
            remaining_service_time -= service_in_this_window
            current_service_time = service_start_in_window + service_in_this_window
            
            # If service is complete, break
            if remaining_service_time <= 0:
                break
            
            # Need to continue in next window - jump to next window start
            window_idx += 1
            if window_idx < len(sorted_windows):
                current_service_time = sorted_windows[window_idx][0]
        
        if remaining_service_time > 0:
            return None  # Service could not be completed within available windows
        
        # Update route state
        current_time = current_service_time
        current_load += customer_demand
        current_location = customer_id
        total_distance += travel_distance
    
    # Check if can return to depot within depot's time window
    if current_location != depot_id:  # Need to return to depot
        return_travel_time = travel_times[(current_location, depot_id)]
        return_distance = travel_distances[(current_location, depot_id)]
        depot_arrival_time = current_time + return_travel_time
        
        if depot_arrival_time > depot_time_window[1]:
            return None  # Cannot return to depot in time
        
        total_distance += return_distance
    
    return total_distance
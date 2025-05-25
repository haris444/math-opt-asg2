import math
from datetime import datetime, timedelta

def time_to_minutes(time_str):
    """Convert time string 'HH:MM' to minutes from midnight."""
    hours, minutes = map(int, time_str.split(':'))
    return hours * 60 + minutes

def load_coordinates(filepath):
    """Load coordinates from instance_coordinates.txt"""
    coordinates = {}
    with open(filepath, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 3:
                node_id = int(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                coordinates[node_id] = (x, y)
    return coordinates

def load_demands(filepath):
    """Load demands from instance_demand.txt"""
    demands = {}
    with open(filepath, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 2:
                customer_id = int(parts[0])
                demand = float(parts[1])
                demands[customer_id] = demand
    return demands

def load_service_times(filepath):
    """Load service times from instance_service_time.txt"""
    service_times = {}
    with open(filepath, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 2:
                customer_id = int(parts[0])
                service_time = float(parts[1])  # in minutes
                service_times[customer_id] = service_time
    return service_times

def load_time_windows(filepath):
    """Load time windows from instance_time_windows.txt"""
    time_windows = {}
    with open(filepath, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 3:
                customer_id = int(parts[0])
                start_time = time_to_minutes(parts[1])
                end_time = time_to_minutes(parts[2])
                
                if customer_id not in time_windows:
                    time_windows[customer_id] = []
                time_windows[customer_id].append((start_time, end_time))
    
    # Sort time windows for each customer
    for customer_id in time_windows:
        time_windows[customer_id].sort()
    
    return time_windows

def calculate_distance_matrix(coordinates):
    """Calculate Euclidean distances between all pairs of locations"""
    distances = {}
    for i in coordinates:
        for j in coordinates:
            if i != j:
                x1, y1 = coordinates[i]
                x2, y2 = coordinates[j]
                dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                distances[(i, j)] = dist
            else:
                distances[(i, j)] = 0.0
    return distances

def calculate_travel_time_matrix(distances, speed=1000):
    """Calculate travel times from distances and speed"""
    travel_times = {}
    for (i, j), distance in distances.items():
        if i != j:
            # Add 1 for triangle inequality as mentioned in the paper
            travel_time = distance / speed  # in hours
            travel_times[(i, j)] = travel_time * 60  # convert to minutes
        else:
            travel_times[(i, j)] = 0.0
    return travel_times

def setup_depot_info(depot_id=1, planning_horizon_hours=24):
    """Setup depot information"""
    return {
        'id': depot_id,
        'time_window': (0, planning_horizon_hours * 60),  # 24 hours in minutes
        'demand': 0,
        'service_time': 0
    }

def load_and_preprocess_data(base_path, instance_name, speed=1000, vehicle_capacity=12600, depot_id=1):
    """
    Load all instance data and perform preprocessing for VRPMTW.
    
    Args:
        base_path (str): Path to directory containing instance files
        instance_name (str): Name prefix of instance files (e.g., 'instance' for instance_*.txt)
        speed (float): Vehicle speed in distance units per hour (default: 1000)
        vehicle_capacity (float): Vehicle capacity in kg (default: 12600)
        depot_id (int): ID of the depot node (default: 1)
    
    Returns:
        dict: Dictionary containing all preprocessed data with the following keys:
            - 'customers_list' (list): List of customer IDs (excludes depot)
            - 'coordinates_data' (dict): {node_id: (x, y)} coordinates for all nodes
            - 'demands' (dict): {customer_id: demand_kg} customer demands in kg
            - 'service_times' (dict): {customer_id: service_time_minutes} service times
            - 'time_windows_data' (dict): {customer_id: [(start_min, end_min), ...]} 
                                         multiple time windows per customer in minutes from midnight
            - 'travel_times' (dict): {(i,j): travel_time_minutes} travel times between all node pairs
            - 'travel_distances' (dict): {(i,j): euclidean_distance} distances between all node pairs
            - 'depot_info' (dict): {'id': depot_id, 'time_window': (0, 1440), 'demand': 0, 'service_time': 0}
            - 'vehicle_capacity' (float): Vehicle capacity in kg
            - 'speed' (float): Vehicle speed in distance units per hour  
            - 'depot_id' (int): ID of the depot node
    
    Example:
        data = load_and_preprocess_data("/content", "instance")
        customers = data['customers_list']  # [2, 3, 4, 5, ...]
        demand_customer_2 = data['demands'][2]  # 2290.0
        travel_time_1_to_2 = data['travel_times'][(1, 2)]  # 15.4 minutes
    """
    print("Loading instance data...")
    
    # Construct file paths
    coords_file = f"{base_path}/{instance_name}_coordinates.txt"
    demand_file = f"{base_path}/{instance_name}_demand.txt"
    service_file = f"{base_path}/{instance_name}_service_time.txt"
    tw_file = f"{base_path}/{instance_name}_time_windows.txt"
    
    # Load raw data
    coordinates = load_coordinates(coords_file)
    demands = load_demands(demand_file)
    service_times = load_service_times(service_file)
    time_windows = load_time_windows(tw_file)
    
    # Set depot service time to 0 if not specified
    if depot_id not in service_times:
        service_times[depot_id] = 0
    
    # Set depot time window (24 hours)
    time_windows[depot_id] = [(0, 24 * 60)]  # 0 to 1440 minutes
    
    # Calculate distance and travel time matrices
    print("Calculating distance and travel time matrices...")
    distances = calculate_distance_matrix(coordinates)
    travel_times = calculate_travel_time_matrix(distances, speed)
    
    # Create customer list (exclude depot)
    customers_list = [node_id for node_id in coordinates.keys() if node_id != depot_id]
    
    # Setup depot info
    depot_info = setup_depot_info(depot_id)
    
    print(f"Loaded {len(customers_list)} customers")
    print(f"Depot: {depot_id}")
    print(f"Vehicle capacity: {vehicle_capacity} kg")
    print(f"Speed: {speed} distance units/hour")
    
    return {
        'customers_list': customers_list,
        'coordinates_data': coordinates,
        'demands': demands,
        'service_times': service_times,
        'time_windows_data': time_windows,
        'travel_times': travel_times,
        'travel_distances': distances,
        'depot_info': depot_info,
        'vehicle_capacity': vehicle_capacity,
        'speed': speed,
        'depot_id': depot_id
    }

# Test function to validate data loading
def print_data_summary(data):
    """Print a summary of loaded data for validation"""
    print("\n=== DATA SUMMARY ===")
    print(f"Number of customers: {len(data['customers_list'])}")
    print(f"Depot ID: {data['depot_info']['id']}")
    print(f"Vehicle capacity: {data['vehicle_capacity']} kg")
    
    # Sample customer info
    sample_customer = data['customers_list'][0] if data['customers_list'] else None
    if sample_customer:
        print(f"\nSample customer {sample_customer}:")
        print(f"  Coordinates: {data['coordinates_data'][sample_customer]}")
        print(f"  Demand: {data['demands'][sample_customer]} kg")
        print(f"  Service time: {data['service_times'].get(sample_customer, 'N/A')} min")
        print(f"  Time windows: {data['time_windows_data'].get(sample_customer, 'N/A')}")
    
    # Check for customers with multiple time windows
    multi_tw_customers = []
    for cust in data['customers_list']:
        if cust in data['time_windows_data'] and len(data['time_windows_data'][cust]) > 1:
            multi_tw_customers.append(cust)
    
    if multi_tw_customers:
        print(f"\nCustomers with multiple time windows: {multi_tw_customers}")
        for cust in multi_tw_customers[:3]:  # Show first 3
            print(f"  Customer {cust}: {data['time_windows_data'][cust]}")
    
    print("===================\n")
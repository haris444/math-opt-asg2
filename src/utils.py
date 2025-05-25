import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def load_vrpmtw_data_txt(coordinates_file, demands_file, service_times_file, time_windows_file):
    """
    Load VRPMTW instance data from text files
    """
    
    # Load coordinates
    coordinates = {}
    with open(coordinates_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 3:
                cust_id = int(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                coordinates[cust_id] = (x, y)
    
    # Load demands
    demands = {}
    with open(demands_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 2:
                cust_id = int(parts[0])
                demand = float(parts[1])
                demands[cust_id] = demand
    
    # Load service times
    service_times = {}
    with open(service_times_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 2:
                cust_id = int(parts[0])
                service_time = float(parts[1])
                service_times[cust_id] = service_time
    
    # Load time windows
    time_windows = {}
    with open(time_windows_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 3:
                cust_id = int(parts[0])
                start_time = time_to_minutes(parts[1])
                end_time = time_to_minutes(parts[2])
                if cust_id not in time_windows:
                    time_windows[cust_id] = []
                time_windows[cust_id].append((start_time, end_time))
    
    return coordinates, demands, service_times, time_windows

def time_to_minutes(time_str):
    """Convert time string (HH:MM) to minutes from start of day"""
    hours, minutes = map(int, time_str.split(':'))
    return hours * 60 + minutes

def visualize_solution(solution, coordinates, demands=None, save_path=None):
    """
    Visualize the VRPMTW solution
    """
    plt.figure(figsize=(12, 10))
    
    # Plot depot
    depot_x, depot_y = coordinates.get(0, (0, 0))
    plt.scatter(depot_x, depot_y, c='red', s=200, marker='s', label='Depot', zorder=5)
    
    # Plot customers
    customer_x = [coordinates[i][0] for i in coordinates if i != 0]
    customer_y = [coordinates[i][1] for i in coordinates if i != 0]
    plt.scatter(customer_x, customer_y, c='blue', s=50, alpha=0.6, label='Customers')
    
    # Plot routes
    colors = plt.cm.rainbow(np.linspace(0, 1, len(solution['routes'])))
    
    for idx, route_info in enumerate(solution['routes']):
        color = colors[idx]
        path_sequence = route_info['path_sequence']
        
        route_x = []
        route_y = []
        
        for i in range(len(path_sequence) - 1):
            curr_node = path_sequence[i]
            next_node = path_sequence[i + 1]
            
            xi, yi = coordinates.get(curr_node, (0, 0))
            xj, yj = coordinates.get(next_node, (0, 0))
            route_x.extend([xi, xj])
            route_y.extend([yi, yj])
        
        plt.plot(route_x, route_y, c=color, linewidth=2, 
                 alpha=0.7, label=f'Vehicle {route_info["vehicle"]}')
        
        # Add arrows to show direction
        for i in range(len(path_sequence) - 1):
            curr_node = path_sequence[i]
            next_node = path_sequence[i + 1]
            
            xi, yi = coordinates.get(curr_node, (0, 0))
            xj, yj = coordinates.get(next_node, (0, 0))
            dx = xj - xi
            dy = yj - yi
            if dx != 0 or dy != 0:
                plt.arrow(xi, yi, dx*0.8, dy*0.8, head_width=200, head_length=100, 
                          fc=color, ec=color, alpha=0.5, length_includes_head=True)
    
    # Add customer labels with demands if available
    for i in coordinates:
        if i != 0:  # Skip depot
            x, y = coordinates[i]
            label = str(i)
            if demands and i in demands:
                label += f"\n({demands[i]:.0f})"
            plt.annotate(label, (x, y), fontsize=8, ha='center', va='bottom')
    
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title(f'VRPMTW Solution - {len(solution["routes"])} vehicles, '
              f'Total distance: {solution["total_distance"]:.2f}')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()

def print_solution_details(solution, coordinates, demands, service_times, time_windows):
    """
    Print detailed solution information
    """
    print("="*60)
    print("VRPMTW SOLUTION DETAILS")
    print("="*60)
    print(f"Number of vehicles used: {solution['num_vehicles']}")
    print(f"Total distance traveled: {solution['total_distance']:.2f}")
    print(f"Objective value: {solution['objective']:.2f}")
    print(f"Optimality gap: {solution['gap']*100:.2f}%")
    print(f"Lower bound: {solution['lower_bound']:.2f}")
    print("="*60)
    
    for route_info in solution['routes']:
        print(f"\nVehicle {route_info['vehicle']}:")
        print("-"*40)
        
        path_sequence = route_info['path_sequence']
        customer_details = route_info['customer_details']
        
        # Calculate route details
        route_demand = sum(demands.get(detail['customer'], 0) for detail in customer_details)
        
        customers_in_route = [detail['customer'] for detail in customer_details]
        
        print(f"Route: {' -> '.join(map(str, path_sequence))}")
        print(f"Number of customers: {len(customers_in_route)}")
        print(f"Total demand: {route_demand:.2f} kg ({route_demand/12600*100:.1f}% of capacity)")
        
        print("Customer details:")
        for detail in customer_details:
            print(f"  - Customer {detail['customer']}: "
                  f"arrive {detail['arrival']:.1f}, wait {detail['wait']:.1f}, "
                  f"service {detail['service_start']:.1f}-{detail['departure']:.1f}, "
                  f"TW: {detail['time_window_chosen']}")

def run_vrpmtw_solver_txt(folder_path="/content"):
    """
    Complete pipeline to load data from text files, solve, and visualize
    """
    # Load data
    coordinates, demands, service_times, time_windows = load_vrpmtw_data_txt(
        f"{folder_path}/instance_coordinates.txt",
        f"{folder_path}/instance_demand.txt",
        f"{folder_path}/instance_service_time.txt",
        f"{folder_path}/instance_time_windows.txt"
    )
    
    print(f"Loaded {len(coordinates)} customers (including depot)")
    print(f"Total demand: {sum(demands.values()):.2f} kg")
    print(f"Minimum vehicles needed: {np.ceil(sum(demands.values())/12600)}")
    
    # Solve with a time limit
    solution = solve_vrpmtw(
        coordinates, demands, service_times, time_windows,
        vehicle_capacity=12600,
        speed=1000/60,  # 16.67 distance units per minute
        max_vehicles=None,  # Let solver determine
        time_limit_seconds=300,  # 5 minutes
        mip_gap=0.05  # 5% gap
    )
    
    if solution and solution['routes']:
        # Print details
        print_solution_details(solution, coordinates, demands, service_times, time_windows)
        
        # Visualize
        visualize_solution(solution, coordinates, demands, "vrpmtw_solution.png")
        
        # Save routes to file
        with open("solution_routes.txt", "w") as f:
            f.write(f"Total vehicles: {solution['num_vehicles']}\n")
            f.write(f"Total distance: {solution['total_distance']:.2f}\n")
            f.write(f"Optimality gap: {solution['gap']*100:.2f}%\n")
            f.write(f"Lower bound: {solution['lower_bound']:.2f}\n\n")
            
            for route_info in solution['routes']:
                path_sequence = route_info['path_sequence']
                f.write(f"Vehicle {route_info['vehicle']}: {' -> '.join(map(str, path_sequence))}\n")
    else:
        print("No solution found!")
    
    return solution
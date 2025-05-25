import math

class Label:
    """
    Represents a label for a partial path in the ESPPRC.

    Attributes:
        current_node_id (int): The last customer visited in the partial path.
        visited_customers_set (set): Set of customer IDs already included in this partial path.
        current_load (float): Total demand accumulated so far.
        dominant_forward_start_intervals (list): A list of tuples, where each tuple is
            (earliest_service_start_time, latest_service_start_time, accumulated_path_distance).
            These are the dominant forward start time intervals at current_node_id.
            'accumulated_path_distance' is the total travel distance from the depot to
            current_node_id leading to this specific start interval.
        accumulated_dual_value_sum (float): Sum of dual variables (pi_i) of visited customers.
        path_sequence (list): Ordered list of customer IDs visited in this partial path.
    """
    def __init__(self,
                 current_node_id: int,
                 visited_customers_set: set,
                 current_load: float,
                 dominant_forward_start_intervals: list, # List of (earliest_start, latest_start, acc_dist)
                 accumulated_dual_value_sum: float,
                 path_sequence: list):
        self.current_node_id = current_node_id
        # Ensure visited_customers_set is a copy to prevent unintended modifications
        self.visited_customers_set = set(visited_customers_set)
        self.current_load = current_load
        # Ensure intervals are sorted by earliest_service_start_time for consistent E_Lf_1 access
        # and for easier processing in dominance and extension logic.
        self.dominant_forward_start_intervals = sorted(dominant_forward_start_intervals, key=lambda x: x[0])
        self.accumulated_dual_value_sum = accumulated_dual_value_sum
        # Ensure path_sequence is a copy
        self.path_sequence = list(path_sequence)

    def get_earliest_start_of_first_interval(self) -> float:
        """
        Returns the earliest start time of the first dominant forward start interval (E_Lf(1) in paper).
        Returns float('inf') if no intervals exist.
        """
        if not self.dominant_forward_start_intervals:
            return float('inf')
        # The first interval is self.dominant_forward_start_intervals[0]
        # Its earliest service start time is the first element of the tuple
        return self.dominant_forward_start_intervals[0][0]

    def __repr__(self):
        # For brevity, dominant_forward_start_intervals might be too long to print fully
        num_intervals = len(self.dominant_forward_start_intervals)
        intervals_repr = (f"[{self.dominant_forward_start_intervals[0]}, ...]"
                          if num_intervals > 0 else "[]")
        return (f"Label(Node: {self.current_node_id}, "
                f"Load: {self.current_load:.2f}, "
                f"Path: {self.path_sequence}, "
                f"Visited: {self.visited_customers_set}, "
                f"#Intervals: {num_intervals}, "
                # f"Top Interval: {self.dominant_forward_start_intervals[0] if num_intervals > 0 else 'N/A'}, "
                f"DualSum: {self.accumulated_dual_value_sum:.2f}, "
                f"E_Lf(1): {self.get_earliest_start_of_first_interval()})")
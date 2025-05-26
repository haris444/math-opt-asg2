class Label:
    """
    Represents a label for a partial path in the ESPPRC (both forward and backward).

    Attributes:
        current_node_id (int): The last/first customer visited (forward/backward).
        visited_customers_set (set): Set of customer IDs already included in this partial path.
        current_load (float): Total demand accumulated so far.
        dominant_intervals (list): List of tuples (earliest_start, latest_start, accumulated_distance).
            For forward: dominant forward start intervals at current_node_id
            For backward: dominant backward start intervals at current_node_id
        accumulated_dual_value_sum (float): Sum of dual variables (pi_i) of visited customers.
        path_sequence (list): Ordered list of customer IDs visited in this partial path.
        is_forward (bool): True for forward label, False for backward label.
    """
    def __init__(self,
                 current_node_id: int,
                 visited_customers_set: set,
                 current_load: float,
                 dominant_forward_start_intervals: list,
                 accumulated_dual_value_sum: float,
                 path_sequence: list,
                 is_forward: bool = True):
        self.current_node_id = current_node_id
        self.visited_customers_set = set(visited_customers_set)
        self.current_load = current_load
        self.dominant_forward_start_intervals = sorted(dominant_forward_start_intervals, key=lambda x: x[0])
        self.accumulated_dual_value_sum = accumulated_dual_value_sum
        self.path_sequence = list(path_sequence)
        self.is_forward = is_forward

    def get_earliest_start_of_first_interval(self) -> float:
        """Returns E_Lf(1) for forward or E_Lb(1) for backward labels."""
        if not self.dominant_forward_start_intervals:
            return float('inf')
        return self.dominant_forward_start_intervals[0][0]

    def get_latest_start_of_last_interval(self) -> float:
        """Returns L_Lf(|F_Lf|) for forward or L_Lb(|B_Lb|) for backward labels."""
        if not self.dominant_forward_start_intervals:
            return -float('inf')
        return self.dominant_forward_start_intervals[-1][1]

    def get_num_intervals(self) -> int:
        """Returns number of dominant intervals."""
        return len(self.dominant_forward_start_intervals)

    def __repr__(self):
        direction = "Forward" if self.is_forward else "Backward"
        return (f"{direction}Label(Node: {self.current_node_id}, "
                f"Load: {self.current_load:.2f}, "
                f"Path: {self.path_sequence}, "
                f"#Intervals: {self.get_num_intervals()}, "
                f"DualSum: {self.accumulated_dual_value_sum:.2f})")
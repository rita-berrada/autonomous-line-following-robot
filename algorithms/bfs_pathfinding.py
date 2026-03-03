#BFS WITH BLOCKED EDGES AND BLOCKED NODES

from collections import deque

def neighbors(pos):
    x, y = pos
    return {
        "N": (x, y + 1),
        "S": (x, y - 1),
        "E": (x + 1, y),
        "W": (x - 1, y),
    }
  
def in_bounds(pos, min_x=0, min_y=0, max_x=4, max_y=4):
    #Check if position is within grid boundaries
    x, y = pos
    return min_x <= x <= max_x and min_y <= y <= max_y

def bfs_path(start, start_dir, end, blocked_nodes=None, blocked_edges=None):

    if blocked_nodes is None:
        blocked_nodes = set()
    if blocked_edges is None:
        blocked_edges = set()

    # If start or end is blocked as a node, no solution
    if start in blocked_nodes or end in blocked_nodes:
        return None

    directions = ["N", "E", "S", "W"]
    
    def turn_left(d):
        return directions[(directions.index(d) - 1) % 4]
    
    def turn_right(d):
        return directions[(directions.index(d) + 1) % 4]

    queue = deque([(start, start_dir, [])])
    visited = {(start, start_dir)}

    while queue:
        current_pos, current_dir, actions = queue.popleft()

        if current_pos == end:
            return actions

        # Helper to test if we can move from A to B
        def can_move(from_pos, to_pos):
            if not in_bounds(to_pos):
                return False
            if to_pos in blocked_nodes:
                return False
            if ((from_pos, to_pos) in blocked_edges or
                (to_pos, from_pos) in blocked_edges):
                return False
            return True

        # 1. Forward : move in current direction
        next_pos_forward = neighbors(current_pos).get(current_dir)
        if next_pos_forward and can_move(current_pos, next_pos_forward):
            state = (next_pos_forward, current_dir)
            if state not in visited:
                visited.add(state)
                queue.append((next_pos_forward, current_dir, actions + ["forward"]))

        # 2. Left: turn left move forward in new direction
        new_dir_left = turn_left(current_dir)
        next_pos_left = neighbors(current_pos).get(new_dir_left)
        if next_pos_left and can_move(current_pos, next_pos_left):
            state = (next_pos_left, new_dir_left)
            if state not in visited:
                visited.add(state)
                queue.append((next_pos_left, new_dir_left, actions + ["left"]))

        # 3. Right: turn right move forward in new direction
        new_dir_right = turn_right(current_dir)
        next_pos_right = neighbors(current_pos).get(new_dir_right)
        if next_pos_right and can_move(current_pos, next_pos_right):
            state = (next_pos_right, new_dir_right)
            if state not in visited:
                visited.add(state)
                queue.append((next_pos_right, new_dir_right, actions + ["right"]))

    return None

def compute_moves(start, start_dir, end, blocked_nodes=None, blocked_edges=None):
    #Returns actions list with 'stop' at the end
    actions = bfs_path(start, start_dir, end, blocked_nodes, blocked_edges)
    if actions is None:
        print("No valid path found.")
        return None
    
    actions.append("stop")
    return actions

# Test
A = compute_moves(start=(4, 4), start_dir="N", end=(4, 0))
print(f"Generated path: {A}")
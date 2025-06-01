#!/usr/bin/env python3
import numpy as np
import random
import os
import sys

# Constants aligned with communal_defines.cpp
BOARD_H = 8
BOARD_W = 8
ACTIONS = ["moveUp", "moveDown", "moveLeft", "moveRight"]
VISITED = -1
EMPTY = 0
SUB = 1
HOSTILE = 2
SURVIVOR = 3
HOME_DIR = os.environ["HOME"]
WORLD_FILE = f"{HOME_DIR}/catkin_ws/src/3806ict_assignment_3/pat/world.csp"
OUTPUT_FILE = f"{HOME_DIR}/catkin_ws/src/3806ict_assignment_3/pat/rl_moves.txt"

# Q-Learning parameters
ALPHA = 0.1  # Learning rate
GAMMA = 0.9  # Discount factor
EPSILON = 0.1  # Exploration rate
NUM_EPISODES = 500
MAX_STEPS = 100

def read_world():
    """Read the current known world from world.csp."""
    with open(WORLD_FILE, 'r') as f:
        lines = f.readlines()
    world = np.zeros((BOARD_H, BOARD_W), dtype=int)
    in_world = False
    row = 0
    for line in lines:
        line = line.strip()
        if line.startswith("var world"):
            in_world = True
            continue
        if in_world and line.startswith('['):
            continue
        if in_world and row < BOARD_H:
            values = line.strip(',').split(',')
            for col, val in enumerate(values):
                if col < BOARD_W:
                    world[row][col] = int(val.strip())
            row += 1
        if line.startswith("var xpos"):
            sub_x = int(line.split('=')[1].strip(';'))
        if line.startswith("var ypos"):
            sub_y = int(line.split('=')[1].strip(';'))
    return world, sub_x, sub_y

def get_next_state(x, y, action, world):
    """Determine the next state based on the action."""
    dx = [-1, 1, 0, 0]  # up, down, left, right
    dy = [0, 0, -1, 1]
    action_idx = ACTIONS.index(action)
    nx, ny = x + dx[action_idx], y + dy[action_idx]
    if (nx < 0 or nx >= BOARD_H or ny < 0 or ny >= BOARD_W or world[nx][ny] == HOSTILE):
        return x, y
    return nx, ny

def get_reward(x, y, target_x, target_y, world):
    """Calculate reward for the current state."""
    if x == target_x and y == target_y:
        return 100.0
    return -1.0

def get_target(task, world, sub_x, sub_y):
    """Select target based on the task."""
    if task == 2:  # GO_HOME
        return 0, 0
    elif task == 1:  # COLLECT_SURVIVORS
        min_dist = float('inf')
        target = (-1, -1)
        for i in range(BOARD_H):
            for j in range(BOARD_W):
                if world[i][j] == SURVIVOR:
                    dist = abs(i - sub_x) + abs(j - sub_y)
                    if dist < min_dist:
                        min_dist = dist
                        target = (i, j)
        return target
    elif task == 0:  # SURVEY_AREA
        min_dist = float('inf')
        target = (-1, -1)
        for i in range(BOARD_H):
            for j in range(BOARD_W):
                if world[i][j] != VISITED and world[i][j] != HOSTILE:
                    dist = abs(i - sub_x) + abs(j - sub_y)
                    if dist < min_dist:
                        min_dist = dist
                        target = (i, j)
        return target
    return (-1, -1)

def train_q_table(world, start_x, start_y, target_x, target_y):
    """Train the Q-table using Q-learning from the current position."""
    Q = np.zeros((BOARD_H, BOARD_W, len(ACTIONS)))
    for _ in range(NUM_EPISODES):
        x, y = start_x, start_y
        for _ in range(MAX_STEPS):
            if (x, y) == (target_x, target_y):
                break
            if random.random() < EPSILON:
                action_idx = random.randint(0, len(ACTIONS)-1)
            else:
                action_idx = np.argmax(Q[x, y])
            action = ACTIONS[action_idx]
            nx, ny = get_next_state(x, y, action, world)
            reward = get_reward(nx, ny, target_x, target_y, world)
            next_max_q = np.max(Q[nx, ny])
            Q[x, y, action_idx] += ALPHA * (reward + GAMMA * next_max_q - Q[x, y, action_idx])
            x, y = nx, ny
            # Encourage exploration by decaying epsilon
            epsilon = max(0.01, EPSILON * 0.995)
    return Q

def get_path(Q, start_x, start_y, target_x, target_y, world):
    """Extract the move sequence from the Q-table."""
    path = []
    x, y = start_x, start_y
    visited = set()
    visited.add((x, y))
    for _ in range(MAX_STEPS):
        if (x, y) == (target_x, target_y):
            break
        action_idx = np.argmax(Q[x, y])
        action = ACTIONS[action_idx]
        path.append(action)
        nx, ny = get_next_state(x, y, action, world)
        if (nx, ny) in visited or (nx, ny) == (x, y):
            break
        x, y = nx, ny
        visited.add((x, y))
    return path

def main(task):
    """Generate moves using RL and write to file."""
    world, sub_x, sub_y = read_world()
    target_x, target_y = get_target(task, world, sub_x, sub_y)
    if target_x == -1:
        with open(OUTPUT_FILE, 'w') as f:
            f.write('<init>')
        return
    Q = train_q_table(world, sub_x, sub_y, target_x, target_y)
    moves = get_path(Q, sub_x, sub_y, target_x, target_y, world)
    with open(OUTPUT_FILE, 'w') as f:
        f.write('<init> ' + ' '.join(moves))
    print(f"Generated {len(moves)} moves for task {task}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: rl_executor.py <task>")
        sys.exit(1)
    task = int(sys.argv[1])
    main(task)
# Lucia Wang
# lucia2@clemson.edu
# 9/14/2023
# CPSC 4420 Assignment 1

# Calls libraries
import itertools
import math
import random
import heapq
from collections import deque

# BFS to find moves from the initial_state to the goal_state
def bfs(initial_state, goal_state):
    # Converts the states passed in and initializes the visited set and queue
    initial_state = tuple(initial_state)
    goal_state = tuple(goal_state)

    visited = set()
    queue = deque()

    queue.append((initial_state, [initial_state], []))

    # Runs until no more states are left
    while len(queue) != 0:
        temp_state, path, move_path = queue.popleft()

        # Prints the states and actions when a path is found
        if temp_state == goal_state:
            for i, state in enumerate(path):
                print(state)
                if i < len(move_path):
                    print("Action: " + str(move_path[i]))
            print("Number of Moves:", len(path) - 1)
            return list(temp_state)

        # Adds the current state to visited
        visited.add(temp_state)

        # Tries all moves and puts into queue if not visited before
        for i in range(4):
            new_state = make_move(list(temp_state), i + 1)
            new_state = tuple(new_state)

            if new_state not in visited and new_state != temp_state:
                new_path = path + [new_state]
                new_move_path = move_path + [i + 1]
                queue.append((new_state, new_path, new_move_path))

    # No solution
    return None

# DFS solution to get from the initial_state to the goal_state
def dfs(initial_state, goal_state):
    # Initializes variables and converts states passed in
    max_iter = 8000
    iter_num = 0
    initial_state = tuple(initial_state)
    goal_state = tuple(goal_state)

    visited = set()
    stack = [(initial_state, [initial_state], [])]

    # Runs until the stack is empty
    while len(stack) != 0:
        # Pops from the stack and adds state to visited
        temp_state, path, moves = stack.pop()
        visited.add(temp_state)

        # Tries all actions
        for i in range(4):
            # Makes a move and sets to new_state
            new_state = make_move(list(temp_state), i + 1)
            new_state = tuple(new_state)

            if new_state not in visited and new_state not in stack:
                # If a valid path is found, prints the states and the moves
                if new_state == goal_state:
                    for j, state in enumerate(path):
                        print(state)
                        if j < len(moves):
                            print("Action: " + str(moves[j]))
                        else:
                            print("Action: " + str(i + 1))
                            print(new_state)
                    print("Number of Moves:", len(path) - 1)
                    return list(temp_state)
                # Otherwise, adds path, moves, and path to the stack
                else:
                    new_path = path + [new_state]
                    new_move_path = moves + [i + 1]
                    stack.append((new_state, new_path, new_move_path))

        # Increases iterations and returns if it goes over
        iter_num = iter_num + 1
        if iter_num >= max_iter:
            return None

    return None

# UCS search where all moves cost 1
def ucs(initial_state, goal_state):
    # Converts states and sets up visited set and priority queue
    initial_state = tuple(initial_state)
    goal_state = tuple(goal_state)

    visited = set()
    priority_queue = [(0, initial_state, [initial_state], [])]

    # Runs until queue is empty
    while len(priority_queue) != 0:
        new_cost, temp_state, path, actions = heapq.heappop(priority_queue)

        # If a valid path is found, prints the state and actions
        if temp_state == goal_state:
            for state, action in zip(path, actions):
                print(state)
                if state != goal_state:
                    print("Action:", action)

            print(goal_state)
            print("Number of Moves:", len(path) - 1)
            return list(temp_state)

        # Adds popped state to visited
        visited.add(temp_state)

        # tries all moves
        for i in range(4):
            new_state = make_move(list(temp_state), i + 1)
            new_state = tuple(new_state)

            # If the state has not been visited, calculates cost and adds it to heap
            if new_state not in visited and new_state != temp_state:
                new_path = path + [new_state]
                new_actions = actions + [i + 1]
                new_cost = new_cost + 1
                heapq.heappush(priority_queue, (new_cost, new_state, new_path, new_actions))

    # No valid path
    return None

# UCS implementation where the costs for each action is stored in costs
def ucs_weights(initial_state, goal_state):
    # Converts states and sets up visited set and priority queue
    costs = [1.5, 0.5, 1, 2]
    initial_state = tuple(initial_state)
    goal_state = tuple(goal_state)

    visited = set()
    priority_queue = [(0, initial_state, [initial_state], [])]

    # Runs while queue is not empty
    while len(priority_queue) != 0:
        cost, temp_state, path, actions = heapq.heappop(priority_queue)

        # If a valid state is found, prints the states and actions
        if temp_state == goal_state:
            for state, action in zip(path, actions):
                print(state)
                if state != goal_state:
                    print("Action:", action)

            print(goal_state)
            print("Number of Moves:", len(path) - 1)
            return list(temp_state)

        # Adds state popped to visited
        visited.add(temp_state)

        # Tries all moves
        for i in range(4):
            new_state = make_move(list(temp_state), i + 1)
            new_state = tuple(new_state)
            # If the state has not been visited, calculates cost and adds it to heap
            if new_state not in visited and new_state != temp_state:
                new_path = path + [new_state]
                new_actions = actions + [i + 1]
                new_cost = cost + costs[i]
                heapq.heappush(priority_queue, (new_cost, new_state, new_path, new_actions))
    # No valid path found
    return None

# Makes a random move from 1 to 4 until divisible state found
def random_moves(initial_state):
    while not is_divisible(initial_state):
        # Gets a random move and moves based off of copy
        temp_state = initial_state.copy()
        rand_action = random.randint(1, 4)
        print(initial_state)
        initial_state = make_move(initial_state, rand_action)
        # Runs until a valid move is made
        while temp_state == initial_state:
            rand_action = random.randint(1, 4)
            initial_state = make_move(initial_state, rand_action)
        print("Action: " + str(rand_action))

    print(initial_state)

# Sets state from user
def set_state():
    avail_states = [0, 1, 2, 3, 4, 5, 6, 7, 8]
    initial_state = []
    # Iterates until 3 x 3 board is filled
    for i in range(9):
        next_ele = int(input("Enter an element in the block: "))
        while next_ele not in avail_states:
            next_ele = int(input("Please enter an integer from 0 to 8: "))
        initial_state.append(next_ele)
        avail_states.remove(next_ele)

    return initial_state

# Checks if rows are divisible by 3
def is_divisible(state):
    temp_state = state.copy()
    while len(temp_state) != 0:
        first_num = (temp_state.pop(0) * 100)
        second_num = (temp_state.pop(0) * 10)
        third_num = temp_state.pop(0)

        row_num = first_num + second_num + third_num
        # Returns false if row is not divisible
        if row_num % 3 != 0:
            return False

    # All rows are divisible by 3
    return True

# Makes a move on the board by swapping 0 and checks bounds of board
def make_move(currState, action):
    index = currState.index(0)
    if action == 1 and index != 0 and index != 1 and index != 2:
        currState[index], currState[index - 3] = currState[index - 3], currState[index]
    if action == 2 and index != 6 and index != 7 and index != 8:
        currState[index], currState[index + 3] = currState[index + 3], currState[index]
    if action == 3 and index != 0 and index != 3 and index != 6:
        currState[index], currState[index - 1] = currState[index - 1], currState[index]
    if action == 4 and index != 2 and index != 5 and index != 8:
        currState[index], currState[index + 1] = currState[index + 1], currState[index]
    else:
        return currState

    return currState

# Generates all states of a 3 x 3 board
def generate_states(dimension):
    states = range(dimension * dimension)
    board_states = list(itertools.permutations(states))
    return board_states

# Prints all states
def print_states(states):
    for board in states:
        print(board)

# Finds 10 random states where odds are not together
def find_random_odds(states, dimension, num_rands):
    random_states = set()
    total_states = math.factorial(dimension * dimension)
    random.seed()

    # Runs until 10 states are found
    while len(random_states) != num_rands:
        index = random.randint(0, total_states - 1)
        temp_board = states[index]

        # Adds board if the odds are not neighbors
        if check_odd_neighbors(temp_board) is True:
            random_states.add(tuple(temp_board))

    return list(random_states)

# Checks if any 2 numbers together are odd
def check_odd_neighbors(temp_board):
    prev_ele = 0
    for element in temp_board:
        # Returns false if 2 odds are found together
        if element % 2 != 0 and prev_ele % 2 != 0:
            return False
        prev_ele = element

    # Returns true at end if no odds were found together
    return True

# UNCOMMENT THE CODE FOR EACH PART
def main():
    dimension = 3
    num_rands = 10
    input_c = [7, 2, 4, 5, 0, 6, 8, 3, 1]
    action = 3

    # PART A SOLUTION
    # board_states = generate_states(int(dimension))
    # print(board_states)

    # PART B SOLUTION
    # board_states = generate_states(int(dimension))
    # random_states = find_random_odds(board_states, dimension, num_rands)
    # print_states(random_states)

    # PART C SOLUTION
    # new_state = make_move(input_c, 3)
    # print(new_state)

    # PART D SOLUTION
    # print("Enter the initial state")
    # initial_state = set_state()
    # random_moves(initial_state)

    # PART E SOLUTION
    # board_states = generate_states(int(dimension))
    # print("Enter the initial state")
    # initial_state = set_state()
    # print("Enter the goal state")
    # goal_state = set_state()
    #
    # moves = bfs(initial_state, goal_state)
    # while moves is None:
    #     total_states = math.factorial(dimension * dimension)
    #     index = random.randint(0, total_states - 1)
    #     initial_state = board_states[index]
    #     moves = bfs(initial_state, goal_state)

    # PART F SOLUTION
    # board_states = generate_states(int(dimension))
    # print("Enter the initial state")
    # initial_state = set_state()
    # print("Enter the goal state")
    # goal_state = set_state()
    #
    # moves = dfs(initial_state, goal_state)
    # while moves is None:
    #     total_states = math.factorial(dimension * dimension)
    #     index = random.randint(0, total_states - 1)
    #     initial_state = board_states[index]
    #     moves = dfs(initial_state, goal_state)
    #
    # bfs(initial_state, goal_state)

    # PART G SOLUTION
    # board_states = generate_states(int(dimension))
    # initial_state = [7, 2, 4, 5, 0, 6, 8, 3, 1]
    # goal_state = [1, 2, 3, 8, 0, 4, 7, 6, 5]
    #
    # moves = bfs(initial_state, goal_state)
    # while moves is None:
    #     total_states = math.factorial(dimension * dimension)
    #     index = random.randint(0, total_states - 1)
    #     initial_state = board_states[index]
    #     moves = bfs(initial_state, goal_state)

    # PART H SOLUTION
    # board_states = generate_states(int(dimension))
    # print("Enter the initial state")
    # initial_state = set_state()
    # print("Enter the goal state")
    # goal_state = set_state()
    #
    # moves = ucs(initial_state, goal_state)
    # while moves is None:
    #     total_states = math.factorial(dimension * dimension)
    #     index = random.randint(0, total_states - 1)
    #     initial_state = board_states[index]
    #     moves = ucs(initial_state, goal_state)
    #
    # ucs_weights(initial_state, goal_state)

if __name__ == "__main__":
    main()

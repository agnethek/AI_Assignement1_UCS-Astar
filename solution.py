from pickle import TRUE
from random import random
import sys
import time
import heapq
from constants import *
from environment import *
from state import State
import queue 
import random
import math
"""
solution.py

This file is a template you should use to implement your solution.

You should implement 

COMP3702 2022 Assignment 1 Support Code

Last updated by njc 01/08/22
"""

class StateNode:
    def __init__(self, environment, state, parent, action_from_parent, path_steps, path_cost):

        self.environment = environment
        self.state = state
        self.parent = parent
        self.action_from_parent = action_from_parent
        self.path_steps = path_steps
        self.path_cost = path_cost

    def get_path(self):
        path = []
        current = self
        while current.action_from_parent is not None:
            path.append(current.action_from_parent)
            current = current.parent
        path.reverse()
        return path

    def get_successors(self):
        successors = []
        for a in ROBOT_ACTIONS:
            success, cost, next_state = self.environment.perform_action(self.state, a)
            if success:
                successors.append(StateNode(self.environment, next_state, self, a, self.path_steps + 1, self.path_cost + cost))
        return successors

    def __str__(self):
        return f"Action from parent: {self.action_from_parent}, parent: {self.parent}, path steps: {self.path_steps}, "

    def __lt__(self, other):
        return self.path_cost < other.path_cost


class Solver:

    def __init__(self, environment, loop_counter):
        self.environment = environment
        self.loop_counter = loop_counter
        #
        # TODO: Define any class instance variables you require here.
        

    def solve_ucs(self):
        """
        Find a path which solves the environment using Uniform Cost Search (UCS).
        :return: path (list of actions, where each action is an element of ROBOT_ACTIONS)
        """
        # For being able to solve this UCS I looked at the tutorial3.py and got a lot of inspiration from their ucs function.
        # Tried to do it on my own, but got stuck and were not able to solve it... 
        
        container = [StateNode(self.environment, self.environment.get_init_state(), None, None, 0, 0)]
        heapq.heapify(container)
        visited = {self.environment.get_init_state(): 0}

        nodes_expanded = 0

        while len(container) > 0:
            self.loop_counter.inc()

            nodes_expanded += 1
            node = heapq.heappop(container)
            

            if self.environment.is_solved(node.state):  ##Here im checking if the state is the goal
                return node.get_path()
        

            successors = node.get_successors()
            
            for i in successors:
                if i.state not in visited.keys() or i.path_cost < visited[i.state]:
                    visited[i.state] = i.path_cost
                    heapq.heappush(container, i)

        return None
        



        pass

    def solve_a_star(self):
        """
        Find a path which solves the environment using A* search.
        :return: path (list of actions, where each action is an element of ROBOT_ACTIONS)
        """
        #
        #
        # TODO: Implement your A* search code here
        x = StateNode(self.environment, self.environment.get_init_state(), None, None, 0, 0)
        container = [(0 + self.euclidean_heuristic(x.state), x)]
        heapq.heapify(container)
        visited = {self.environment.get_init_state(): 0}
        nodes_expanded = 0

        while len(container) > 0:
            self.loop_counter.inc()
            nodes_expanded += 1
            _, node = heapq.heappop(container)

            if self.environment.is_solved(node.state):  ##Here im checking if the state is the goal
                return node.get_path()

            successors = node.get_successors()
            for i in successors:
                if i.state not in visited.keys() or i.path_cost < visited[i.state]:
                    visited[i.state] = i.path_cost
                    heapq.heappush(container, (i.path_cost + self.euclidean_heuristic(i.state), i))
        return None


        pass
    
    #
    #
    # TODO: Add any additional methods here
    #

    def manhattan_heuristic(self, state):
        environment = self.environment
        target_list = self.environment.target_list
        current_pos = state.robot_posit
        cost = 0

        for i in target_list:
            cost += (abs(i[0]-current_pos[0]) + abs(i[1] - current_pos[1]))
        
        return (cost/len(target_list))         ##Divide it on the length of the targetlist
    
    def euclidean_heuristic(self, state):
        environment = self.environment
        target_list = self.environment.target_list
        current_pos = state.robot_posit
        cost = 0

        for i in target_list:
            cost += (i[0]-current_pos[0])**2 + (i[1] - current_pos[1])**2
            cost = math.sqrt(cost)
        return cost

    def hamming_distance(self, state):
        environment = self.environment
        target_list = self.environment.target_list
        current_pos = state.robot_posit
        dist_counter = 0

        for i in target_list:
            dist = abs((i[0]-current_pos[0]) + (i[1] - current_pos[1]))
            if dist != 0:
                dist_counter += 1
            else:
                dist_counter = dist_counter
        return dist_counter




  

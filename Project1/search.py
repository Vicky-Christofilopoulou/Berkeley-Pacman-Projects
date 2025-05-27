# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
from inspect import stack

import util
from game import Directions
from typing import List

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """

    # For the implementation of DFS I used the same steps as the algorith presented in class
    # from unit 2, presentation blind1spp.pdf from pages 53 using algorith GRAPH-SEARCH and as
    # a fringe we use a stack.

    # From the presentation graph_search.pdf pages 16, each node will store:
    # 1. state          -- node[0]
    # 2. parent node    -- node[1]
    # 3. action         -- node[2]
    # 4. path cost      -- node[3]
    #

    from util import Stack
    fringe = Stack()                # In fringe the item was last pushed is the first out - LIFO

    explored_set = set()            # A list where we store the visited states
    path = []                       # The path from the start to the goal

    # Before we start the algorith we check if start node is also goal node
    node = (problem.getStartState(), None, None, 0)
    if problem.isGoalState(node[0]):
        return path

    fringe.push((node, path))               # Because it is not the goal node, we add the node and the path to reach it

    while not fringe.isEmpty():     # This means we still have neighbours to explore
        popped_node, popped_path = fringe.pop()

        if problem.isGoalState(popped_node[0]):      # We return the path from start node to the goal node
            return popped_path

        if popped_node[0] not in explored_set:    # We haven't visited yet
            explored_set.add(popped_node[0])  # We add the popped node to the ones we have visited
            successors = problem.getSuccessors(popped_node[0])
            # print(successors);

            if successors is None:
                print(f"No successors found for node: {popped_node}")
                continue    # We skip to the next iteration

            for i in successors:
                if i[0] not in explored_set:
                    newpath = popped_path + [i[1]]
                    # node (state, parent node, action, cost)
                    node = (i[0], popped_node[0], i[1], popped_node[3] + i[2])
                    fringe.push((node, newpath))
                    # print("fringe", fringe)
                    # print("explored_set", explored_set)

    return []       # We return empty if no solution is found
    util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    # For the implementation of BFS I used the same steps as DFS, but as a fringe we use a Queue.

    # From the presentation graph_search.pdf pages 16, each node will store:
    # 1. state          -- node[0]
    # 2. parent node    -- node[1]
    # 3. action         -- node[2]
    # 4. path cost      -- node[3]
    #

    from util import Queue
    fringe = Queue()                # In fringe the item was first in is the first out - FIFO

    explored_set = set()            # A list where we store the visited states
    path = []                       # The path from the start to the goal

    # Before we start the algorith we check if start node is also goal node
    node = (problem.getStartState(), None, None, 0)
    if problem.isGoalState(node[0]):
        return path

    fringe.push((node, path))               # Because it is not the goal node, we add the node and the path to reach it

    while not fringe.isEmpty():     # This means we still have neighbours to explore
        popped_node, popped_path = fringe.pop()

        if problem.isGoalState(popped_node[0]):      # We return the path from start node to the goal node
            return popped_path

        if popped_node[0] not in explored_set:    # We haven't visited yet
            explored_set.add(popped_node[0])  # We add the popped node to the ones we have visited
            successors = problem.getSuccessors(popped_node[0])
            # print(successors);

            if successors is None:
                print(f"No successors found for node: {popped_node}")
                continue    # We skip to the next iteration

            for i in successors:
                if i[0] not in explored_set:
                    newpath = popped_path + [i[1]]
                    # node (state, parent node, action, cost)
                    node = (i[0], popped_node[0], i[1], popped_node[3] + i[2])
                    fringe.push((node, newpath))
                    # print("fringe", fringe)
                    # print("explored_set", explored_set)

    return []       # We return empty if no solution is found
    util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""

    # For the implementation of UCS I used the same steps as DFS, but as a fringe we use a Priority Queue
    # and we add the priority for each node.

    # From the presentation graph_search.pdf pages 16, each node will store:
    # 1. state          -- node[0]
    # 2. parent node    -- node[1]
    # 3. action         -- node[2]
    # 4. path cost      -- node[3]
    #

    from util import PriorityQueue
    fringe = PriorityQueue()  # Now, the fringe is a priority queue

    explored_set = set()  # A list where we store the visited states
    path = []  # The path from the start to the goal

    priority_dic = {}

    # Before we start the algorith we check if start node is also goal node
    node = (problem.getStartState(), None, None, 0)
    priority = 0  # Because it is the start
    priority_dic[node[0]] = priority

    if problem.isGoalState(node[0]):
        return path

    fringe.push((node, path),priority)  # Because it is not the goal node, we add the node and the path to reach it

    while not fringe.isEmpty():  # This means we still have neighbours to explore
        popped_node, popped_path = fringe.pop()

        if problem.isGoalState(popped_node[0]):  # We return the path from start node to the goal node
            return popped_path

        explored_set.add(popped_node[0])  # We add the popped node to the ones we have visited
        successors = problem.getSuccessors(popped_node[0])
        # print(successors);

        if successors is None:
            print(f"No successors found for node: {popped_node}")
            continue  # We skip to the next iteration

        for i in successors:
            if i[0] not in explored_set:
                newpath = popped_path + [i[1]]
                # node (state, parent node, action, cost)
                cost = popped_node[3] + i[2]
                node = (i[0], popped_node[0], i[1], cost)
                fringe.push((node, newpath), cost)

                # Update priority dictionary
                priority_dic[i[0]] = cost

                explored_set.add(i[0])

            elif priority_dic[i[0]] > popped_node[3] + i[2]:
                cost = popped_node[3] + i[2]
                node = (i[0], popped_node[0], i[1], cost)
                newpath = popped_path + [i[1]]
                fringe.update((node, newpath), cost)
                priority_dic[i[0]] = cost

    return []  # We return empty if no solution is found

    util.raiseNotDefined()

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""

    # For the implementation of A* I used the same steps as DFS, but as a fringe we use a Priority Queue
    # and we add the priority for each node.
    # The priority is calculated f(n) = g(n) + h(n) which is cost of path + cost of heuristic

    # From the presentation graph_search.pdf pages 16, each node will store:
    # 1. state          -- node[0]
    # 2. parent node    -- node[1]
    # 3. action         -- node[2]
    # 4. path cost      -- node[3]
    #

    from util import PriorityQueue
    fringe = PriorityQueue()  # Now, the fringe is a priority queue

    explored_set = set()  # A list where we store the visited states
    path = []  # The path from the start to the goal

    priority_dic = {}

    # Before we start the algorith we check if start node is also goal node
    node = (problem.getStartState(), None, None, 0)
    priority_dic[node[0]] = heuristic(node[0], problem)

    if problem.isGoalState(node[0]):
        return path

    priority = 0  + heuristic(node[0], problem)      # Because it is the start
    fringe.push((node, path), priority)  # Because it is not the goal node, we add the node and the path to reach it

    while not fringe.isEmpty():  # Continue until the fringe is empty
        popped_node, popped_path = fringe.pop()

        if problem.isGoalState(popped_node[0]):  # Check if the popped node is the goal
            return popped_path  # Return the path to the goal node

        explored_set.add(popped_node[0])  # Add the popped node to explored set
        successors = problem.getSuccessors(popped_node[0])
        # print("successors",successors)

        if successors is None:
            print(f"No successors found for node: {popped_node}")
            continue  # Skip to the next iteration if no successors

        for i in successors:  # Iterate over each successor
            if i[0] not in explored_set:
                newpath = popped_path + [i[1]]
                cost = popped_node[3] + i[2]
                node = (i[0], popped_node[0], i[1], cost)
                priority = cost + heuristic(node[0], problem)
                fringe.push((node, newpath), priority)

                # Update priority dictionary
                priority_dic[i[0]] = priority

                explored_set.add(i[0])
            elif priority_dic[i[0]] > popped_node[3] + i[2] + heuristic(i[0], problem):
                cost = popped_node[3] + i[2]
                node = (i[0], popped_node[0], i[1], cost)
                newpath = popped_path + [i[1]]
                priority = cost + heuristic(i[0], problem)
                fringe.update((node, newpath), priority)
                priority_dic[i[0]] = priority

    return []  # Return empty if no solution is found

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch






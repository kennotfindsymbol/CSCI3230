"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from sys import path
import util

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


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """ 
    Question 1: Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    You only need to submit this file. Do not change other files!
    If you finish this function, you almost finish all the questions!
    Read util.py to find suitable data structure!
    All you need is pass all the code in commands.txt
    """

    # SOLUTION 1 iterative function
    "*** YOUR CODE HERE ***"

    # start = problem.getStartState()
    # start_path = []
    # visited = []
    # node_stack = util.Stack()
    # path_stack = util.Stack()
    # node_stack.push(start)
    # path_stack.push(start_path)
    # while not node_stack.isEmpty():
    #     node = node_stack.pop()
    #     node_path = path_stack.pop()
    #     if problem.isGoalState(node):
    #         print(len(node_path))
    #         return node_path
    #     for v, action, _ in problem.getSuccessors(node):
    #         if v not in visited:
    #             visited.append(v)
    #             node_stack.push(v)
    #             path_to_be_pushed = node_path.copy()
    #             path_to_be_pushed.append(action)
    #             path_stack.push(path_to_be_pushed)

    

    # SOLUTION 1 recursive function
    "*** YOUR CODE HERE ***"


    visited_recursive = []
    start_recursive = problem.getStartState()
    path_recursive = []
    ret_recursive = []
    def recursiveDFS(node, path):
        if problem.isGoalState(node):
            print('found', end=' ')
            ret_recursive.append(path)
        visited_recursive.append(node)
        for v, action, _ in problem.getSuccessors(node):
            if v not in visited_recursive:
                if len(ret_recursive) != 0:
                    return
                if problem.isGoalState(v):
                    ret_recursive.append(path + [action])
                    return 
                recursiveDFS(v, path + [action])             
    recursiveDFS(start_recursive, path_recursive)  
    return ret_recursive[0]
    
    


def breadthFirstSearch(problem):
    """Question 2: Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    start_path = []
    visited = []
    node_queue = util.Queue()
    path_queue = util.Queue()
    node_queue.push(start)
    path_queue.push(start_path)
    while not node_queue.isEmpty():
        node = node_queue.pop()
        node_path = path_queue.pop()
        if problem.isGoalState(node):
            print(len(node_path))
            return node_path
        if node not in visited:
            visited.append(node)
            for v, action, _ in problem.getSuccessors(node):
                node_queue.push(v)
                path_to_be_pushed = node_path.copy()
                path_to_be_pushed.append(action)
                path_queue.push(path_to_be_pushed)

    
    
    


def uniformCostSearch(problem):
    """Question 3: Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    ret = []
    ret_cost = float('inf')
    start = problem.getStartState()
    start_path = []
    start_cost = 0
    visited = []
    node_queue = util.PriorityQueue()
    path_queue = util.PriorityQueue()
    cost_queue = util.PriorityQueue()
    node_queue.push(start, start_cost)
    path_queue.push(start_path, start_cost)
    cost_queue.push(start_cost, start_cost)
    while not node_queue.isEmpty():
        node = node_queue.pop()
        node_path = path_queue.pop()
        node_cost = cost_queue.pop()
        if problem.isGoalState(node):
            print(len(node_path))
            # if node_cost < ret_cost:
            #     ret = node_path
            return node_path

        if node not in visited:
            visited.append(node)
            for v, action, cost in problem.getSuccessors(node):
                cost_to_be_pushed = node_cost + cost
                cost_queue.push(cost_to_be_pushed, cost_to_be_pushed)
                node_queue.push(v, cost_to_be_pushed)
                path_to_be_pushed = node_path.copy()
                path_to_be_pushed.append(action)
                path_queue.push(path_to_be_pushed, cost_to_be_pushed)
                
    return ret


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Question 4: Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    ret = []
    ret_cost = float('inf')
    start = problem.getStartState()
    start_path = []
    start_cost = 0
    start_cost_plus_heuristic = start_cost + heuristic(start, problem)
    visited = []
    node_queue = util.PriorityQueue()
    path_queue = util.PriorityQueue()
    cost_queue = util.PriorityQueue()
    node_queue.push(start, start_cost_plus_heuristic)
    path_queue.push(start_path, start_cost_plus_heuristic)
    cost_queue.push(start_cost, start_cost_plus_heuristic)
    while not node_queue.isEmpty():
        node = node_queue.pop()
        node_path = path_queue.pop()
        node_cost = cost_queue.pop()
        if problem.isGoalState(node):
            print(len(node_path))
            # if node_cost < ret_cost:
            #     ret = node_path
            return node_path

        if node not in visited:
            visited.append(node)
            for v, action, cost in problem.getSuccessors(node):
                cost_to_be_pushed = node_cost + cost
                cost_plus_heuristic = cost_to_be_pushed + heuristic(v, problem)
                cost_queue.push(cost_to_be_pushed, cost_plus_heuristic)
                node_queue.push(v, cost_plus_heuristic)
                path_to_be_pushed = node_path.copy()
                path_to_be_pushed.append(action)
                path_queue.push(path_to_be_pushed, cost_plus_heuristic)
                
    return ret



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

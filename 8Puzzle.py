# Jacob Gulan
import numpy as np
import os

class Node:
    # Class that calculates the cost values and finds the path from the
    # current state to the goal state
  
    def __init__(self, v, g, f):
        # Initializes cost functions grid array
        self.value = v  # Grid
        self.gn = g     # Path cost (g)
        self.fn = f     # Estimated cost of the cheapest solution through n
        self.hn = 0     # Estimated cost of the cheapest path from the state at node n to a goal state

    def updateHeuristic(self, htype, goal):
        
        # Assign heuristic value
        if htype == 0:
            self.hn = self.manhattan(goal)
        elif htype == 1:
            self.hn = self.misplaced(goal)
        else:
            return 'Error'
        
        # Update cost of cheapest solution
        self.fn = self.gn + self.hn
        
    
    def getNeighbors(self):
        # Retrieve the neighboring nodes of the zero node

        # Find the position of the zero node
        y, x = np.argwhere(self.value == 0)[0][:]
        
        # Retrieve coordinates of possible neighbors
        coords = np.array([[y-1, x], [y+1, x], 
                           [y, x-1], [y, x+1]])
        neighbors = []

        # Ensure -1 doesn't go to end of array
        if (y-1 == -1):
            np.delete(coords, np.argwhere(coords == [y-1, x]))
        if (x-1 == -1):
            np.delete(coords, np.argwhere(coords == [y, x-1]))

        # Find possible neighboring nodes and add them to the neighbors array
        for i in coords:
            ny, nx = i 
            if self.isInBounds(nx, ny):
                # Change the location of the zero node
                newPuzzle = []
                newPuzzle = np.copy(self.value)
                temp = newPuzzle[ny][nx]
                newPuzzle[ny][nx] = newPuzzle[y][x]
                newPuzzle[y][x] = temp             

                # Add new node to the neighbors array
                neighbor = Node(newPuzzle, self.gn + 1, 0)
                neighbors.append(neighbor)
        
        return neighbors

    def isInBounds(self, x, y):
        # Determines if coordinates are in bounds for the grid
        return x >= 0 and x < len(self.value) and y >= 0 and y < len(self.value)

    def manhattan(self, goal):
        # Manhattan distance heurstic
        nodes = list(goal.flatten())
        cost = 0

        # Search through each element in the grid and determine the cost
        # of the heuristic
        for i in range (0, len(self.value)):
            for j in range (0, len(self.value)):
                index = nodes.index(self.value[i][j])
                xGoal, yGoal = index // 3, index % 3

                # Calculate the cost of the heuristic
                if self.value[i][j] != '0':
                    cost += (abs(xGoal - i) + abs(yGoal - j))
        return cost
    
    def misplaced(self, goal):
        # Misplaced tiles heuristic

        cost = 0
        for i in range(0, len(self.value)):
            for j in range(0, len(self.value)):
                # If a node that's not the zero node is out of position add
                # cost to the heuristic
                if self.value[i][j] != goal[i][j] and self.value[i][j] != '0':
                    cost += 1
        
        return cost

class AStarSearch:
    # Class that allows the user to input an initial state, goal state, and
    # begins the search while printing results

    def __init__(self):
        # Initializes frontier and explored arrays
        self.openList = []      # frontier
        self.closedList = []    # explored

    def main(self):
        # Method to initiate A* Search

        # Initialize variables
        initialGrid = []
        goalGrid = []

        # User inputs initial state
        user = input("Input initial state in a similar format of '0 1 2 3 4 5 6 7 8': ")
        user = user.split()
        initialGrid = np.array([user[0:3], user[3:6], user[6:9]])
        initialGrid = initialGrid.astype(np.int)

        # User inputs goal state
        user = input("Input goal state in a similar format of '0 1 2 3 4 5 6 7 8': ")
        user = user.split()
        goalGrid = np.array([user[0:3], user[3:6], user[6:9]])
        goalGrid = goalGrid.astype(np.int)

        # User inputs heuristic type
        htype = int(input("Input 0 for Manhattan and 1 for Misplaced Tiles: "))

        initialState = Node(initialGrid, 0, 0)

        # Ensure either 0 or 1 was entered as a heuristic type
        if (htype == 0 or htype == 1):
            initialState.updateHeuristic(htype, goalGrid)
        else:
            return "Please enter valid heuristic type: 0 or 1."
        
        self.openList.append(initialState)

        count = 0
        # Begin A* Search
        while True:
            # Break if solution can't be found
            if count > 100:
                print("\n\nNot Solvable")
                return
            count += 1

            curr = self.openList[0]
            
            # Create transition between grids
            print("")

            # Print grid
            for i in curr.value:
                for j in i:
                    print(j, end = " ")
                print("")

            # Check to see if goal state is met
            if htype == 0:
                if (curr.manhattan(goalGrid) == 0):
                    break
            else:
                if (curr.misplaced(goalGrid) == 0):
                    break

            # Add neighboring nodes to frontier
            for i in curr.getNeighbors():
                i.updateHeuristic(htype, goalGrid)
                self.openList.append(i)
            
            # Add previously worked node to explored
            self.closedList.append(curr)
            del self.openList[0]

            # Sort frontier based on cost
            self.openList.sort(key = lambda x:x.fn, reverse = False)
        
        # Print frontier and explored lists
        print("\n\nGenerated Nodes: ", len(self.openList))
        print("Expanded Nodes: ", len(self.closedList))

# Execute the program
search = AStarSearch()
search.main()
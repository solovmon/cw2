import rospy

from collections import defaultdict
from explorer_node_base import ExplorerNodeBase
from comp0037_reactive_planner_controller.occupancy_grid import OccupancyGrid

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNodeF(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.blackList = []

    def updateFrontiers(self):

        #dictionaries used to store 'labels'
        openMapQueue = defaultdict(bool)
        closedMapQueue = defaultdict(bool)
        openFrontierQueue = defaultdict(bool)
        closedFrontierQueue = defaultdict(bool)

        searchQ = []
        allFrontiers = []

        start = (0, 0)

        searchQ.append(start)
        openMapQueue[start] = True

        #outer BFS for finding the first frontier cell
        while len(searchQ) > 0:
            cell = searchQ.pop(0)

            #if cell has been looked at before (search), skip it
            if closedMapQueue[cell] is True:
                continue
            
            if self.isFrontierCell(cell[0],cell[1]) is True:
                frontier = []
                frontierQ = []
                frontierQ.append(cell)
                openFrontierQueue[cell] = True
                
                #inner BFS for finding cells on the same frontier
                while len(frontierQ) > 0:
                    frontierCell = frontierQ.pop(0)

                    #if cell has been looked at before, skip it
                    if (closedMapQueue[frontierCell] == True) \
                    | (closedFrontierQueue[frontierCell] == True):
                        continue
                    
                    if self.isFrontierCell(frontierCell[0],frontierCell[1]) is True:
                        #add cell to the frontier and mark it as looked at as part of a frontier
                        frontier.append(frontierCell)

                        #look at neighbouring cells and add them to the frontier search queue if elegible
                        for neighbour in self.getNeighbourCells(frontierCell):
                            if (closedMapQueue[neighbour] == False)    \
                            |  (closedFrontierQueue[neighbour] == False) \
                            |  (openFrontierQueue[neighbour] == False)      :
                                frontierQ.append(neighbour)
                                openFrontierQueue[neighbour] = True

                    #mark cell as looked at (frontier)
                    closedFrontierQueue[frontierCell] = True

                #add frontier to the frontiers array
                allFrontiers.append(frontier)

                #mark all cells part of the frontier as looked at (search)
                for frontierCell in frontier:
                    closedMapQueue[frontierCell] = True
            
            #look at neighbouring cells and add them to the outer search queue if elegible
            for neighbour in self.getNeighbourCells(cell):
                if (closedMapQueue[neighbour] == True)  \
                |  (openMapQueue[neighbour]== True)       :
                    for neighbour2 in self.getNeighbourCells(neighbour):
                        if openMapQueue[neighbour2] is True:
                            #add neigbour to the outer search queue and mark as such
                            searchQ.append(neighbour)
                            openMapQueue[neighbour] = True
                            break

        #save froniers
        self.frontiers = allFrontiers
        print(allFrontiers)

        #if no frontiers found, end the program
        if len(allFrontiers) == 0:
            return False

        return True

    def chooseNewDestination(self):
        candidateGood = False
        destination = None
        smallestD2 = float('inf')

        for frontier in self.frontiers:
            for candidate in frontier:
                candidateGood = True
                for k in range(0, len(self.blackList)):
                    if self.blackList[k] == candidate:
                        candidateGood = False
                        break
                if candidateGood is True:
                    d2 = candidate[0]**2+(candidate[1]-0.5*self.occupancyGrid.getHeightInCells())**2
                    if (d2 < smallestD2):
                        destination = candidate
                        smallestD2 = d2

        return candidateGood, destination

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
            self.blackList.append(goal)

    def getNeighbourCells(self, cell):
        neighbours = []
        cardinals = [(-1,-1),(-1,0),(-1,-1),(1,-1),(1,0),(1,1),(0,-1),(0,1)]
        for card in cardinals:
            neighbour = (cell[0] + card[0], cell[1] + card[1])
            if (neighbour[0] >= 0) & (neighbour[0] < self.occupancyGrid.getWidthInCells()) \
            & (neighbour[1] >= 0) & (neighbour[1] < self.occupancyGrid.getHeightInCells()) :
                neighbours.append(neighbour)
        return neighbours


            

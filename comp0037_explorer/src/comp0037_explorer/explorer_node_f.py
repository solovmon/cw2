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
        explored = defaultdict(bool)
        searchQ = []
        allFrontiers = []

        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                if self.occupancyGrid.getCell(x, y) == 0:
                    start = (x, y)
                    print(start)
                    print('START')
                    break
            else: 
                continue
            break   

        searchQ.append(start)

        #search through the available cells.
        while len(searchQ) > 0:
            print(searchQ)
            cell = searchQ.pop(0)
            explored[cell] = True
            if self.isFrontierCell(cell[0],cell[1]) is True:
                frontier = []
                frontier.append(cell)
                frontierQ = []
                frontierQ.append(cell)
                print('FRONTIER')
                while len(frontierQ) > 0:
                    print(frontierQ)
                    frontierCell = frontierQ.pop(0)
                    explored[frontierCell] = True
                    for neighbour in self.getNeighbourCells(frontierCell):
                        if explored[neighbour] is False:
                            if self.isFrontierCell(neighbour[0],neighbour[1]) is True:
                                frontier.append(neighbour)
                                frontierQ.append(neighbour)
                
                allFrontiers.append(frontier)

            for neighbour in self.getNeighbourCells(cell):
                if explored[neighbour] is False:
                    searchQ.append(neighbour)

        print('XD')
        print(allFrontiers)

        if len(allFrontiers) == 0:
            return False
        
        self.frontiers = allFrontiers

        

        return True

    def chooseNewDestination(self):


#         print 'blackList:'
#         for coords in self.blackList:
#             print str(coords)

        candidateGood = False
        destination = None
        smallestD2 = float('inf')

        for frontier in self.frontiers:
            for candidate in frontier:
                d2 = candidate[0]**2+(candidate[1]-0.5*self.occupancyGrid.getHeightInCells())**2
                if (d2 < smallestD2):
                    destination = candidate
                    smallestD2 = d2

        return candidateGood, destination

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)

    def getNeighbourCells(self, cell):
        neighbours = []
        cardinals = [(-1,-1),(-1,0),(-1,-1),(1,-1),(1,0),(1,1),(0,-1),(0,1)]
        for card in cardinals:
            neighbour = (cell[0] + card[0], cell[1] + card[1])
            if (neighbour[0] >= 0) & (neighbour[0] < self.occupancyGrid.getWidthInCells()) \
            & (neighbour[1] >= 0) & (neighbour[1] < self.occupancyGrid.getHeightInCells()) \
            & (self.occupancyGrid.getCell(neighbour[0], neighbour[1]) == 0):
                neighbours.append(neighbour)

        return neighbours


            

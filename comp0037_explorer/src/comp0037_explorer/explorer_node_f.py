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
        openFrontierQueue[start]   = False
        closedFrontierQueue[start] = False
        openMapQueue[start]        = True
        closedMapQueue[start]      = False

        #outer BFS for finding the first frontier cell
        while len(searchQ) > 0:
            cell = searchQ.pop(0)
            #print('searchQ:', searchQ)

            #if cell has been looked at before (search), skip it
            if closedMapQueue[cell] is True:
                continue
            
            if self.isFrontierCell(cell[0],cell[1]) is True:
                frontier = []
                frontierQ = []
                frontierQ.append(cell)

                openFrontierQueue[cell]   = True
                closedFrontierQueue[cell] = False
                openMapQueue[cell]        = False
                closedMapQueue[cell]      = False
                
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
                            &  (closedFrontierQueue[neighbour] == False) \
                            &  (openFrontierQueue[neighbour] == False)      :
                                frontierQ.append(neighbour)

                                #mark cell as added to the frontier queue
                                openFrontierQueue[neighbour]   = True
                                closedFrontierQueue[neighbour] = False
                                openMapQueue[neighbour]        = False
                                closedMapQueue[neighbour]      = False

                    #mark cell as looked at (frontier)
                    openFrontierQueue[frontierCell]   = False
                    closedFrontierQueue[frontierCell] = True
                    openMapQueue[frontierCell]        = False
                    closedMapQueue[frontierCell]      = False
                

                #add frontier to the frontiers array
                allFrontiers.append(frontier)

                #mark all cells part of the frontier as looked at (search)
                for frontierCell in frontier:
                    openFrontierQueue[frontierCell]   = False
                    closedFrontierQueue[frontierCell] = False
                    openMapQueue[frontierCell]        = False
                    closedMapQueue[frontierCell]      = True
            
            #look at neighbouring cells and add them to the outer search queue if elegible
            for neighbour in self.getNeighbourCells(cell):
                #print('neighbour:', neighbour)
                if (closedMapQueue[neighbour] == False)  \
                &  (openMapQueue[neighbour]== False)       :
                    for neighbour2 in self.getNeighbourCells(neighbour):
                        #print('neighbour2:', neighbour2)
                        if openMapQueue[neighbour2] is True:
                            #add neigbour to the outer search queue and mark as such
                            searchQ.append(neighbour)
                            #print('neighbour:', neighbour)
                            openFrontierQueue[neighbour]   = False
                            closedFrontierQueue[neighbour] = False
                            openMapQueue[neighbour]        = True
                            closedMapQueue[neighbour]      = False
                            break

            #mark cell as looked at (search)
            openFrontierQueue[cell]   = False
            closedFrontierQueue[cell] = False
            openMapQueue[cell]        = False
            closedMapQueue[cell]      = True

        #save froniers
        self.frontiers = allFrontiers
        print('found frontiers:,',allFrontiers)

        #if no frontiers found, end the program
        if len(allFrontiers) == 0:
            return False
        return True

    def chooseNewDestination(self):
        candidateGood = False
        destination = None

        # #comment this out for largest frontier
        # smallestD2 = float('inf')
        # if len(self.frontiers) > 0:
        #     for frontier in self.frontiers:
        #         if len(frontier) > 1:
        #             middlePoint = frontier[len(frontier)//2]
        #             position = self.current_position
        #             d2 = (position.x - middlePoint[0])**2 + (position.y - middlePoint[1])**2
        #             if (d2 < smallestD2):
        #                 choosenFrontier = frontier
        #                 smallestD2 = d2
        
        # length = len(choosenFrontier)
        # #looping through the closest frontier from the middle cell in both directions
        # for x in range (length//2, length):
        #     if choosenFrontier[x] not in self.blackList:
        #         candidateGood = True
        #         destination = choosenFrontier[x]
        #         break
        
        # if candidateGood is not True:
        #     for x in range (-length//2, 1):
        #         if choosenFrontier[-x] not in self.blackList:
        #             candidateGood = True
        #             destination = choosenFrontier[-x]
        #             break
        # #end   

        #comment this out for closest frontier
        largestFrontier = []
        if len(self.frontiers) > 0:
            for frontier in self.frontiers:
                if len(frontier) > len(largestFrontier):
                    largestFrontier = frontier
                    length = len(largestFrontier)

        #looping through the largest frontier from the middle cell in both directions
        for x in range (length//2, length):
            if largestFrontier[x] not in self.blackList:
                candidateGood = True
                destination = largestFrontier[x]
                break
        
        if candidateGood is not True:
            for x in range (-length//2, 1):
                if largestFrontier[-x] not in self.blackList:
                    candidateGood = True
                    destination = largestFrontier[-x]
                    break
        # #end

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


            

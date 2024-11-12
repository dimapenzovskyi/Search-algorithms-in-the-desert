import math, heapq
import numpy as np


def heuristic(a, b, hchoice):
    if hchoice == 1:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    if hchoice == 2:
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def blocked(cX, cY, dX, dY, matrix):
    if cX + dX < 0 or cX + dX >= matrix.shape[0]:
        return True
    if cY + dY < 0 or cY + dY >= matrix.shape[1]:
        return True
    if dX != 0 and dY != 0:
        if matrix[cX + dX][cY] == 1 and matrix[cX][cY + dY] == 1:
            return True
        if matrix[cX + dX][cY + dY] == 1:
            return True
    else:
        if dX != 0:
            if matrix[cX + dX][cY] == 1:
                return True
        else:
            if matrix[cX][cY + dY] == 1:
                return True
    return False


def dblock(cX, cY, dX, dY, matrix):
    if matrix[cX - dX][cY] == 1 and matrix[cX][cY - dY] == 1:
        return True
    else:
        return False


def direction(cX, cY, pX, pY):
    dX = int(math.copysign(1, cX - pX))
    dY = int(math.copysign(1, cY - pY))
    if cX - pX == 0:
        dX = 0
    if cY - pY == 0:
        dY = 0
    return (dX, dY)


def nodeNeighbours(cX, cY, parent, matrix):
    neighbours = []
    if type(parent) != tuple:
        for i, j in [
            (-1, 0),
            (0, -1),
            (1, 0),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]:
            if not blocked(cX, cY, i, j, matrix):
                neighbours.append((cX + i, cY + j))

        return neighbours
    dX, dY = direction(cX, cY, parent[0], parent[1])

    if dX != 0 and dY != 0:
        if not blocked(cX, cY, 0, dY, matrix):
            neighbours.append((cX, cY + dY))
        if not blocked(cX, cY, dX, 0, matrix):
            neighbours.append((cX + dX, cY))
        if (
            not blocked(cX, cY, 0, dY, matrix)
            or not blocked(cX, cY, dX, 0, matrix)
        ) and not blocked(cX, cY, dX, dY, matrix):
            neighbours.append((cX + dX, cY + dY))
        if blocked(cX, cY, -dX, 0, matrix) and not blocked(
            cX, cY, 0, dY, matrix
        ):
            neighbours.append((cX - dX, cY + dY))
        if blocked(cX, cY, 0, -dY, matrix) and not blocked(
            cX, cY, dX, 0, matrix
        ):
            neighbours.append((cX + dX, cY - dY))

    else:
        if dX == 0:
            if not blocked(cX, cY, dX, 0, matrix):
                if not blocked(cX, cY, 0, dY, matrix):
                    neighbours.append((cX, cY + dY))
                if blocked(cX, cY, 1, 0, matrix):
                    neighbours.append((cX + 1, cY + dY))
                if blocked(cX, cY, -1, 0, matrix):
                    neighbours.append((cX - 1, cY + dY))

        else:
            if not blocked(cX, cY, dX, 0, matrix):
                if not blocked(cX, cY, dX, 0, matrix):
                    neighbours.append((cX + dX, cY))
                if blocked(cX, cY, 0, 1, matrix):
                    neighbours.append((cX + dX, cY + 1))
                if blocked(cX, cY, 0, -1, matrix):
                    neighbours.append((cX + dX, cY - 1))
    return neighbours


def jump(cX, cY, dX, dY, matrix, goal):

    nX = cX + dX
    nY = cY + dY
    if blocked(nX, nY, 0, 0, matrix):
        return None

    if (nX, nY) == goal:
        return (nX, nY)
        
    oX = nX
    oY = nY

    if dX != 0 and dY != 0:
        while True:
            if (
                not blocked(oX, oY, -dX, dY, matrix)
                and blocked(oX, oY, -dX, 0, matrix)
                or not blocked(oX, oY, dX, -dY, matrix)
                and blocked(oX, oY, 0, -dY, matrix)
            ):
                return (oX, oY)

            if (
                jump(oX, oY, dX, 0, matrix, goal) != None
                or jump(oX, oY, 0, dY, matrix, goal) != None
            ):
                return (oX, oY)

            oX += dX
            oY += dY

            if blocked(oX, oY, 0, 0, matrix):
                return None

            if dblock(oX, oY, dX, dY, matrix):
                return None

            if (oX, oY) == goal:
                return (oX, oY)
    else:
        if dX != 0:
            while True:
                if (
                    not blocked(oX, nY, dX, 1, matrix)
                    and blocked(oX, nY, 0, 1, matrix)
                    or not blocked(oX, nY, dX, -1, matrix)
                    and blocked(oX, nY, 0, -1, matrix)
                ):
                    return (oX, nY)

                oX += dX

                if blocked(oX, nY, 0, 0, matrix):
                    return None

                if (oX, nY) == goal:
                    return (oX, nY)

        else:
            while True:
                if (
                    not blocked(nX, oY, 1, dY, matrix)
                    and blocked(nX, oY, 1, 0, matrix)
                    or not blocked(nX, oY, -1, dY, matrix)
                    and blocked(nX, oY, -1, 0, matrix)
                ):
                    return (nX, oY)

                oY += dY

                if blocked(nX, oY, 0, 0, matrix):
                    return None

                if (nX, oY) == goal:
                    return (nX, oY)

    return jump(nX, nY, dX, dY, matrix, goal)


def identifySuccessors(cX, cY, came_from, matrix, goal):
    successors = []
    neighbours = nodeNeighbours(cX, cY, came_from.get((cX, cY), 0), matrix)

    for cell in neighbours:
        dX = cell[0] - cX
        dY = cell[1] - cY

        jumpPoint = jump(cX, cY, dX, dY, matrix, goal)

        if jumpPoint != None:
            successors.append(jumpPoint)

    return successors

def method(matrix, start, goals, hchoice):
    start, _ = start
    came_from = {}
    close_set = set()
    gscore = {start: 0}
    fscore = {start: min(heuristic(start, goal, hchoice) for goal in goals)}
    
    pqueue = []
    heapq.heappush(pqueue, (fscore[start], start))
    
    while pqueue:
        current = heapq.heappop(pqueue)[1]
        
        if current in goals:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        close_set.add(current)
        
        for goal in goals:
            successors = identifySuccessors(current[0], current[1], came_from, matrix, goal)
            
            for successor in successors:
                if successor in close_set:
                    continue
                
                tentative_g_score = gscore[current] + lenght(current, successor, hchoice)
                
                if tentative_g_score < gscore.get(successor, float('inf')):
                    came_from[successor] = current
                    gscore[successor] = tentative_g_score
                    # Dynamically update heuristic based on closest remaining goal
                    fscore[successor] = tentative_g_score + min(heuristic(successor, g, hchoice) for g in goals)
                    heapq.heappush(pqueue, (fscore[successor], successor))
    
    return None


def search(problem, hchoice=1):
    matrix = np.array(problem.map)
    start = problem.initial
    goals = problem.lake_goals

    jumpPoints = method(matrix, start, goals, hchoice)
    if not jumpPoints:
        return None

    # current = start
    # result = [(None, current)]
    # for jumpPoint in jumpPoints[1:]:
    #     while current[0] != jumpPoint:
    #         if abs(current[0][0] - jumpPoint[0]) > abs(current[0][1] - jumpPoint[1]):
    #             if current[0][0] - jumpPoint[0] > 0:
    #                 current = problem.result(current, 'UP')
    #                 result += [('UP', current)]
    #             else:
    #                 current = problem.result(current, 'DOWN')
    #                 result += [('DOWN', current)]
    #         else:
    #             if current[0][1] - jumpPoint[1] > 0:
    #                 current = problem.result(current, 'LEFT')
    #                 result += [('LEFT', current)]
    #             elif current[0][1] - jumpPoint[1] < 0:
    #                 current = problem.result(current, 'RIGHT')
    #                 result += [('RIGHT', current)]
    #         if current[0] in problem.lake_goals:
    #             return result
            
    # return result
    return jumpPoints

def lenght(current, jumppoint, hchoice):
    dX, dY = direction(current[0], current[1], jumppoint[0], jumppoint[1])
    dX = math.fabs(dX)
    dY = math.fabs(dY)
    lX = math.fabs(current[0] - jumppoint[0])
    lY = math.fabs(current[1] - jumppoint[1])
    if hchoice == 1:
        if dX != 0 and dY != 0:
            lenght = lX * 14
            return lenght
        else:
            lenght = (dX * lX + dY * lY) * 10
            return lenght
    if hchoice == 2:
        return math.sqrt(
            (current[0] - jumppoint[0]) ** 2 + (current[1] - jumppoint[1]) ** 2
        )

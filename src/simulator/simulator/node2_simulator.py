import rclpy
import numpy as np
from ast import literal_eval
from rclpy.node import Node
from tkinter import *
from std_msgs.msg import String
from PIL import Image, ImageTk
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import math
from math import pow, atan2, sqrt, sin, cos, remainder, tau, pi
import time
import heapq
import numpy as np
import math
import numpy as np

import math
from typing import List

class Vertex:
    def __init__(self, pos: (int, int)):
        self.pos = pos
        self.edges_and_costs = {}

    def edges_and_c_old(self):
        return self.edges_and_costs

class Vertices:
    def __init__(self):
        self.list = []

    def add_vertex(self, v: Vertex):
        self.list.append(v)

    def vertices(self):
        return self.list

def getHeuristicVal(p: (int, int), q: (int, int)) -> float:
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)

def getSuccessors(x: int, y: int, map):
    successors = []
    if x<9:
        if map[x + 1][y + 0] == 0:
            successors.append((x + 1, y + 0))
    if y<9:
        if map[x + 0][y + 1] == 0:
            successors.append((x + 0, y + 1))
    if x>1:
        if map[x - 1][y + 0] == 0:
            successors.append((x - 1, y + 0))
    if y>1:
        if map[x + 0][y - 1] == 0:
            successors.append((x + 0, y - 1))
    if y<9 and x<9:
        if map[x + 1][y + 1] == 0:
            successors.append((x + 1, y + 1))
    if x>1 and y<9:
        if map[x - 1][y + 1] == 0:
            successors.append((x - 1, y + 1))
    if x>1 and y>1:
        if map[x - 1][y - 1] == 0:
            successors.append((x - 1, y - 1))
    if x<9 and y>1:
        if map[x + 1][y - 1] == 0:
            successors.append((x + 1, y - 1))

    return(successors)

class Priority:
    """
    handle lexicographic order of keys
    """
    def __init__(self, k1, k2):
        self.k1 = k1
        self.k2 = k2

    def __lt__(self, other):
        """
        lexicographic 'lower than'
        :param other: comparable keys
        :return: lexicographic order
        """
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 < other.k2)

    def __le__(self, other):
        """
        lexicographic 'lower than or equal'
        :param other: comparable keys
        :return: lexicographic order
        """
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 <= other.k2)

    def get(self):
        return self.k1, self.k2

class PriorityNode:
    """
    handle lexicographic order of vertices
    """

    def __init__(self, priority, vertex):
        """
        :param priority: the priority of a
        :param vertex:
        """
        self.priority = priority
        self.vertex = vertex

    def __le__(self, other):
        """
        :param other: comparable node
        :return: lexicographic order
        """
        return self.priority <= other.priority

    def __lt__(self, other):
        """
        :param other: comparable node
        :return: lexicographic order
        """
        return self.priority < other.priority
    
    def get(self):
        return self.vertex, self.priority 

class PriorityQueue:
    def __init__(self):
        self.heap = []
        self.vertices_in_heap = []

    def top(self):
        return self.heap[0].vertex

    def top_key(self):
        if len(self.heap) == 0: return Priority(float('inf'), float('inf'))
        return self.heap[0].priority

    def pop(self):
        lastelt = self.heap.pop()  
        self.vertices_in_heap.remove(lastelt)
        if self.heap:
            returnitem = self.heap[0]
            self.heap[0] = lastelt
            self._siftup(0)
        else:
            returnitem = lastelt
        return returnitem

    def insert(self, vertex, priority):
        item = PriorityNode(priority, vertex)
        self.vertices_in_heap.append(vertex)
        self.heap.append(item)
        self._siftdown(0, len(self.heap) - 1)

    def remove(self, vertex):
        self.vertices_in_heap.remove(vertex)
        for index, priority_node in enumerate(self.heap):
            if priority_node.vertex == vertex:
                self.heap[index] = self.heap[len(self.heap) - 1]
                self.heap.remove(self.heap[len(self.heap) - 1])
                break
        self.build_heap()

    def update(self, vertex, priority):
        for index, priority_node in enumerate(self.heap):
            if priority_node.vertex == vertex:
                self.heap[index].priority = priority
                break
        self.build_heap()

    def build_heap(self):
        n = len(self.heap)
        for i in reversed(range(n // 2)):
            self._siftup(i)

    def _siftdown(self, startpos, pos):
        newitem = self.heap[pos]
        while pos > startpos:
            parentpos = (pos - 1) >> 1
            parent = self.heap[parentpos]
            if newitem < parent:
                self.heap[pos] = parent
                pos = parentpos
                continue
            break
        self.heap[pos] = newitem

    def _siftup(self, pos):
        endpos = len(self.heap)
        startpos = pos
        newitem = self.heap[pos]
        # Bubble up the smaller child until hitting a leaf.
        childpos = 2 * pos + 1  # leftmost child position
        while childpos < endpos:
            # Set childpos to index of smaller child.
            rightpos = childpos + 1
            if rightpos < endpos and not self.heap[childpos] < self.heap[rightpos]:
                childpos = rightpos
            # Move the smaller child up.
            self.heap[pos] = self.heap[childpos]
            pos = childpos
            childpos = 2 * pos + 1
        # The leaf at pos is empty now.  Put newitem there, and bubble it up
        # to its final resting place (by sifting its parents down).
        self.heap[pos] = newitem
        self._siftdown(startpos, pos)

OBSTACLE = 255
UNOCCUPIED = 0

class DstarLite:
    def __init__(self):
        self.new_edges_and_old_costs = None
        self.init = False

    def Init(self, s_start, s_goal):
        self.s_start = s_start
        self.s_goal = s_goal
        self.s_last = s_start
        self.k_m = 0  # accumulation
        self.U = PriorityQueue()
        self.rhs = np.ones(self.map.shape) * np.inf
        self.g = self.rhs.copy()
        self.rhs[self.s_goal] = 0
        self.U.insert(self.s_goal, Priority(getHeuristicVal(self.s_start, self.s_goal), 0))
        self.init = True

    def SetMap(self, _map):
        self.map = _map.T

    def CalculateKey(self, s: (int, int)):
        k1 = min(self.g[s], self.rhs[s]) + getHeuristicVal(self.s_start, s) + self.k_m
        k2 = min(self.g[s], self.rhs[s])
        return Priority(k1, k2)

    def getCost(self, u: (int, int), v: (int, int)):
        if self.map[u[0]][u[1]] == 1 or self.map[v[0]][v[1]] == 1:
            return float('inf')
        else:
            return getHeuristicVal(u, v)

    def contain(self, u: (int, int)) -> (int, int):
        return u in self.U.vertices_in_heap

    def UpdateVertex(self, u: (int, int)):
        if self.g[u] != self.rhs[u] and self.contain(u):
            self.U.update(u, self.CalculateKey(u))
        elif self.g[u] != self.rhs[u] and not self.contain(u):
            self.U.insert(u, self.CalculateKey(u))
        elif self.g[u] == self.rhs[u] and self.contain(u):
            self.U.remove(u)

    def ComputeShortestPath(self):

        while self.U.top_key() < self.CalculateKey(self.s_start) or self.rhs[self.s_start] > self.g[self.s_start]:
            u = self.U.top()
            k_old = self.U.top_key()
            k_new = self.CalculateKey(u)

            if k_old < k_new:
                self.U.update(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                self.U.remove(u)
                pred = getSuccessors(u[0], u[1], self.map)
                for s in pred:
                    if s != self.s_goal:
                        self.rhs[s] = min(self.rhs[s], self.getCost(s, u) + self.g[u])
                    self.UpdateVertex(s)

            else:
                self.g_old = self.g[u]
                self.g[u] = float('inf')
                pred = getSuccessors(u[0], u[1], self.map)

                pred.append(u)
                for s in pred:
                    if self.rhs[s] == self.getCost(s, u) + self.g_old:
                        if s != self.s_goal:
                            min_s = float('inf')
                            succ = getSuccessors(s[0], s[1], self.map)
                            for s_ in succ:
                                temp = self.getCost(s, s_) + self.g[s_]
                                if min_s > temp:
                                    min_s = temp
                            self.rhs[s] = min_s
                    self.UpdateVertex(u)

    def rescan(self) -> Vertices:
        new_edges_and_old_costs = self.new_edges_and_old_costs
        self.new_edges_and_old_costs = None
        return new_edges_and_old_costs

    def move_and_replan(self, robot_position: (int, int)):
        path = [robot_position]
        self.s_start = robot_position
        self.s_last = self.s_start
        self.ComputeShortestPath()

        while self.s_start != self.s_goal:
            assert (self.rhs[self.s_start] != float('inf')), "There is no known path!"
            succ = getSuccessors(self.s_start[0], self.s_start[1], self.map)

            min_s = float('inf')
            arg_min = None
            for s_ in succ:
                temp = self.getCost(self.s_start, s_) + self.g[s_]

                if temp < min_s:
                    min_s = temp
                    arg_min = s_

            self.s_start = arg_min
            path.append(self.s_start)
            changed_edges_with_old_cost = self.rescan()

            if changed_edges_with_old_cost:
                self.k_m += getHeuristicVal(self.s_last, self.s_start)
                self.s_last = self.s_start
                vertices = changed_edges_with_old_cost.vertices
                for vertex in vertices:
                    v = vertex.pos
                    succ_v = vertex.edges_and_c_old
                    for u, c_old in succ_v.items():
                        c_new = self.getCost(u, v)
                        if c_old > c_new:
                            if u != self.s_goal:
                                self.rhs[u] = min(self.rhs[u], self.c(u, v) + self.g[v])
                        elif self.rhs[u] == c_old + self.g[v]:
                            if u != self.s_goal:
                                min_s = float('inf')
                                succ_u = getSuccessors(self.u[0], self.u[1], self.map)
                                for s_ in succ_u:
                                    temp = self.c(u, s_) + self.g[s_]
                                    if min_s > temp:
                                        min_s = temp
                                self.rhs[u] = min_s
                            self.UpdateVertex(u)
            self.ComputeShortestPath()

        print("path found!", path)
        return path, self.g, self.rhs

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.winH = 400
        self.winW = 400
        window = Tk()
        self.canvas = Canvas(window, width=self.winW, height=self.winH, borderwidth=0, highlightthickness=0)
        # self.file_path = 'pngwing.png'
        self.file_path = "/home/bolka/dev_ws/src/pngwing.png"
        self.img_p = PhotoImage(file=self.file_path)

        self.s_goal = None, None

        self.positionx = 320
        self.positiony = 80
        self.goal_pose_x = self.positionx
        self.goal_pose_y = self.positiony 
        self.theta = 0
        self.flag = False
        self.first_time = True
        self.L = 1

        self.map = None
        self.dstar = DstarLite()

        self.replane = False
        self.path = None
        self.side_x = 40
        self.side_y = 40
        self.s_start = (int(self.positionx// self.side_x),int(self.positiony// self.side_x))
        self.s_goal = (int(self.goal_pose_x// self.side_x),int(self.goal_pose_y// self.side_x))

    def rotatedPhotoImage(self, img, angle):
        angleInRads = angle
        diagonal = sqrt(img.width()**2 + img.height()**2)
        xmidpoint = img.width()/2
        ymidpoint = img.height()/2
        newPhotoImage = PhotoImage(width=int(diagonal), height=int(diagonal))
        for x in range(img.width()):
            for y in range(img.height()):
                xnew = float(x)
                ynew = float(-y)
                xnew = xnew - xmidpoint
                ynew = ynew + ymidpoint
                xnew, ynew = xnew*cos(angleInRads) - ynew*sin(angleInRads), xnew * sin(angleInRads) + ynew*cos(angleInRads)
                xnew = xnew + diagonal/2
                ynew = ynew - diagonal/2
                xnew = xnew
                ynew = -ynew
                rgb = '#%02x%02x%02x' % img.get(x, y)
                newPhotoImage.put(rgb, (int(xnew), int(ynew)))
                newPhotoImage.put(rgb, (int(xnew+1), int(ynew)))
        return newPhotoImage

    def listener_callback(self, msg):
        canvas=self.canvas

        map_values = np.array(literal_eval(msg.data))
        self.dstar.SetMap(map_values)

        self.ncols_map, self.nrows_map = len(map_values[0]), len(map_values)
        self.cellW = self.winW / self.ncols_map
        self.cellH = self.winH / self.nrows_map

        self.side_x = self.winH/self.ncols_map
        self.side_y = self.winW/self.nrows_map
        img = self.rotatedPhotoImage(self.img_p, -self.theta)
        self.image = canvas.create_image(self.positionx, self.positiony, image=img)
        canvas.tag_raise(self.image)
        canvas.update()

        class Node:
            def __init__(self, row, col, val):
                self.row = row
                self.col = col
                self.val = val
                return

        def generatGrid():
            grid = []
            for r in range(self.nrows_map):
                row = [ Node(r, c, map_values[r][c]) for c in range(self.ncols_map) ]
                grid.append(row)
            return grid

        def drawNode(canvas, node):
            x1 = self.cellW * node.col
            y1 = self.cellH * node.row
            x2 = x1 + self.cellW
            y2 = y1 + self.cellH
            if node.val==1:
                canvas.create_rectangle(x1, y1, x2, y2, fill='gray')
            else:
                canvas.create_rectangle(x1, y1, x2, y2, fill='white')
            return

        def drawGrid(canvas, grid):
            for row in grid:
                for node in row:
                    drawNode(canvas, node)
            return

        def left_click(event):
            y_pos = int(event.x // self.side_x)
            x_pos = int(event.y // self.side_y)

            grid = self.grid
            if grid[x_pos][y_pos].val == 1:
                grid[x_pos][y_pos].val  = 0
                map_values[x_pos][y_pos] = 0
            elif grid[x_pos][y_pos].val == 0:
                grid[x_pos][y_pos].val  = 1
                map_values[x_pos][y_pos] = 1

            data_file = '/home/bolka/dev_ws/src/data.csv'
            # data_file = 'data.csv'
            np.savetxt(data_file, map_values, delimiter=',')
        
        def right_click(event):
            print("right click")
            y_pos = int(event.x // self.side_x)
            x_pos = int(event.y // self.side_y)
            if self.grid[x_pos][y_pos].val == 0:
                self.goal_pose_x = event.x
                self.goal_pose_y = event.y
            if self.goal_pose_x != self.positionx and self.goal_pose_y!=self.positiony:
                self.s_start = (int(self.positionx// self.side_x),int(self.positiony// self.side_x))
                self.s_goal = (int(self.goal_pose_x// self.side_x),int(self.goal_pose_y// self.side_x))
                self.dstar.Init(self.s_start, self.s_goal)

        canvas.bind("<Button-1>", left_click)
        canvas.bind("<Button-3>", right_click)

        self.grid = generatGrid()
        drawGrid(canvas, self.grid)
        canvas.pack(side = 'top') 
        if self.dstar.init:
            self.path, g, rhs = self.dstar.move_and_replan((int(self.positionx//self.side_x), int(self.positiony//self.side_y)))
            self.drawPath(self.path, self.canvas)
            self.dstar.Init(self.s_start, self.s_goal)
            self.path = self.follow_path()

    def drawPath(self, array, canvas):
        print(array)
        for step in array:
            x1 = self.cellW * step[0]
            y1 = self.cellH * step[1]
            x2 = x1 + self.cellW
            y2 = y1 + self.cellH
            canvas.create_rectangle(x1, y1, x2, y2, fill='green')

    def follow_path(self):
        if len(self.path)>1:
            coord = self.path[1] 
        else:
            coord = self.path[0] 
        goal = (coord[0] * self.side_x+20, coord[1] * self.side_y+20)
        move = self.move2goal(self.canvas, self.image, self.grid, goal)
        pth = self.path.remove(coord)
        return pth

    def update_pose(self, new_x, new_y, new_theta, canvas, image):
        self.positionx = new_x
        self.positiony = new_y
        self.theta = new_theta

        canvas.delete(self.image)
        img = PhotoImage(file=self.file_path)
        img = self.rotatedPhotoImage(img,-(self.theta))
        self.image = canvas.create_image(self.positionx, self.positiony, image=img)
        canvas.tag_raise(self.image)
        canvas.update()

    def euclidean_distance(self, goal):
        return sqrt(pow((goal[0] - self.positionx), 2) +
                    pow((goal[1] - self.positiony), 2))

    def linear_vel(self,goal, constant=1.2):
        return constant * self.euclidean_distance(goal)

    def steering_angle(self, goal):
        return atan2(goal[1]  - self.positiony, goal[0]  - self.positionx)

    def angular_vel(self, goal, constant=12):
        return constant * (self.steering_angle(goal) - self.theta)

    def check_walls(self, grid, new_x, new_y, new_theta, canvas, image):
        x_pos = int((new_x) // self.side_x)
        y_pos = int(new_y // self.side_y)
        if grid[y_pos][x_pos].val == 1:
                self.update_pose(new_x=self.positionx, new_y=self.positiony, new_theta=self.theta+pi, canvas=canvas, image=self.image)
                return True
        return False

    def move2goal(self, canvas, image, grid, goal):
        if self.euclidean_distance(goal) > 2:
            time.sleep(0.05)
            v = float(self.linear_vel(goal))
            w = self.angular_vel(goal)
            dleft = (v - 1/2*self.L*w) * 0.1
            dright = (v + 1/2*self.L*w) * 0.1 
            dcenter = (dleft + dright) / 2
            phi = (dright - dleft) / self.L

            delta_x = dcenter * cos(self.theta) 
            delta_y = dcenter * sin(self.theta)
            delta_th = self.theta + phi
            new_x = self.positionx+delta_x
            new_y = self.positiony+delta_y
            new_theta = delta_th

            if self.check_walls(grid, new_x, new_y, new_theta, canvas, image):
                print("stopping")
                v=0
                w=0
                self.goal = (self.positionx, self.positiony)
                walls = True

            self.update_pose(new_x=new_x, new_y=new_y, new_theta=new_theta, canvas=canvas, image=image)
        self.v=0;
        self.w=0;
        print('stopped')

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
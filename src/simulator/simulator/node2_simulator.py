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



class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.
    """
    def  __init__(self):
        self.heap = []
        # self.vertices_in_heap = []

    def insert(self, vertex, priority):
        heapq.heappush(self.heap, (vertex, priority))
        # self.vertices_in_heap.append(vertex)

    def pop(self):
        (vertex, _) = heapq.heappop(self.heap)
        return vertex

    def top(self):
        return self.heap[0]

    def topKey(self):
        return self.heap[0][1]

    #neviem ci je dobre lebo neviem co presne ma robit 
    def remove(self,p):
        rem = (float('-inf'),float('-inf'))
        self.update(p, rem)
        a = self.pop()

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, vertex, priority):
        # If item already in priority queue with higher priority, update its priority and rebuild the heap.
        # If item already in priority queue with equal or lower priority, do nothing.
        # If item not in priority queue, do the same thing as self.push.
        for index, temp in enumerate(self.heap):
            if temp[0] == vertex:
                if temp[1] <= priority:
                    break
                del self.heap[index]
                print(priority)
                self.heap.append((vertex, priority))
                heapq.heapify(self.heap)
                break
        else:
            self.insert(vertex, priority)


def heuristic(p: (int, int), q: (int, int)) -> float:
    """
    Helper function to compute distance between two points.
    :param p: (x,y)
    :param q: (x,y)
    :return: manhattan distance
    """
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)


class MinimalSubscriber(Node):
    
    def __init__(self):
        
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.winH = 400
        self.winW = 400
        window = Tk()
        self.canvas = Canvas(window, width=self.winW, height=self.winH, borderwidth=0, highlightthickness=0)

        self.goal_pose_x = 310
        self.goal_pose_y = 310

        self.img_p = PhotoImage(file="/home/bolka/dev_ws/src/pngwing.png")
        self.positionx = 110
        self.positiony = 110
        self.theta = 0
        self.flag = False
        self.first_time = True
        self.L = 1

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
        self.ncols_map, self.nrows_map= len(map_values[0]), len(map_values)

        cellW = self.winW / self.ncols_map
        cellH = self.winH / self.nrows_map

        self.side_x = self.winH/self.ncols_map
        self.side_y = self.winW/self.nrows_map

       
        img = self.rotatedPhotoImage(self.img_p, -self.theta)

        image = canvas.create_image(self.positionx, self.positiony, image=img)
        canvas.tag_raise(image)
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
            x1 = cellW * node.col
            y1 = cellH * node.row
            x2 = x1 + cellW
            y2 = y1 + cellH
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

            np.savetxt('/home/bolka/dev_ws/src/data.csv', map_values, delimiter=',')

        def right_click(event):
            y_pos = int(event.x // self.side_x)
            x_pos = int(event.y // self.side_y)
            if self.grid[x_pos][y_pos].val == 0:
                self.goal_pose_x = event.x
                self.goal_pose_y = event.y

        canvas.bind("<Button-1>", left_click)
        canvas.bind("<Button-3>", right_click)

        self.grid = generatGrid()
        drawGrid(canvas, self.grid)
        canvas.pack(side = 'top') 
        

        self.myMain(canvas, image, self.grid, map_values)     


    def update_pose(self, new_x, new_y, new_theta, canvas, image):
        self.positionx = new_x
        self.positiony = new_y
        self.theta = new_theta

        print(self.positionx, self.positiony, self.theta)
        canvas.delete(image)
        img = PhotoImage(file="/home/bolka/dev_ws/src/pngwing.png")
        img = self.rotatedPhotoImage(img,-(self.theta))
        image = canvas.create_image(self.positionx, self.positiony, image=img)
        canvas.tag_raise(image)
        canvas.update()
        
    def initialize_dstarlite(self, map_values):
        """
        :param map: the ground truth map of the environment provided by gui
        :param s_start: start location
        :param s_goal: end location
        """
        # self.new_edges_and_old_costs = None

        # algorithm start
        self.s_start = [int(self.positionx// self.side_x),int(self.positiony// self.side_x)]
        self.s_goal = [int(self.goal_pose_x// self.side_x),int(self.goal_pose_y// self.side_x)]
        self.s_last = self.s_start
        self.k_m = 0  # accumulation
        self.U = PriorityQueue()
        self.rhs = np.ones((self.ncols_map, self.nrows_map)) * np.inf
        self.g = self.rhs.copy()

        self.sensed_map = map_values

        self.rhs[self.s_goal[0]][self.s_goal[1]] = 0

        self.U.insert(self.s_goal, (heuristic(self.s_start, self.s_goal), 0))


    # #posielat len prvu cast vertexu
    def calculate_key(self, s: [int, int]):
        """
        :param s: the vertex we want to calculate key
        :return: Priority class of the two keys
        """
        print('this is it', self.s_start, s )
        k1 = min(self.g[s[0]][s[1]], self.rhs[s[0]][s[1]]) + heuristic(self.s_start, s) + self.k_m
        k2 = min(self.g[s[0]][s[1]], self.rhs[s[0]][s[1]])
        return (k1, k2)

    # def contain(self, u: (int, int)) -> (int, int):
    #     return u in self.U.vertices_in_heap
    # todo maybe add contain 

    def update_vertex(self, u: (int, int)):
        u=u[0]
        # ak je v PQ a je inkonzistetny tak ho updatujes
        if self.g[u[0]][u[1]]!= self.rhs[u[0]][u[1]]:
            self.U.update(u, self.calculate_key(u))
        # ak  nie je v PQ tak ho vlozis
        elif self.g[u[0]][u[1]] != self.rhs[u[0]][u[1]]:
            self.U.insert(u, self.calculate_key(u))
        # ak je v PQ a je konzisentny tak ho zmazes 
        elif self.g[u[0]][u[1]] == self.rhs[u[0]][u[1]]:
            self.U.remove(u)

    def c(self, u: (int, int), v: (int, int), map_values) -> float:
        """
        calcuclate the cost between nodes
        :param u: from vertex
        :param v: to vertex
        :return: euclidean distance to traverse. inf if obstacle in path
        """
        if map_values[u[0]][u[1]] == 1 or map_values[v[0]][v[1]]:
            return float('inf')
        else:
            return heuristic(u, v)

    def compute_shortest_path(self, map_values):
        # print(self.U.topKey(), self.calculate_key(self.s_start))
        # print(self.rhs[self.s_start[0]][self.s_start[1]], self.g[self.s_start[0]][self.s_start[1]]) 

        while self.U.topKey() < self.calculate_key(self.s_start) or self.rhs[self.s_start[0]][self.s_start[1]] > self.g[self.s_start[0]][self.s_start[1]]:
            u = self.U.top()
            k_old = self.U.topKey()
            k_new = self.calculate_key(u[0])
            print(u)
            # print('this', self.c([0,0],[2,2], map_values))

            if k_old < k_new:
                print('update')
            #     self.U.update(u, k_new)

            elif self.g[u[0][0]][u[0][1]] > self.rhs[u[0][0]][u[0][1]]:
                print('remove')
            #     self.g[u] = self.rhs[u]
            #     self.U.remove(u)
            #     pred = self.sensed_map.succ(vertex=u)
            #     for s in pred:
            #         if s != self.s_goal:
            #             self.rhs[s] = min(self.rhs[s], self.c(s, u) + self.g[u])
            #         self.update_vertex(s)
            else:
                print('nic')
            #     self.g_old = self.g[u]
            #     self.g[u] = float('inf')
            #     pred = self.sensed_map.succ(vertex=u)
            #     pred.append(u)
            #     for s in pred:
            #         if self.rhs[s] == self.c(s, u) + self.g_old:
            #             if s != self.s_goal:
            #                 min_s = float('inf')
            #                 succ = self.sensed_map.succ(vertex=s)
            #                 for s_ in succ:
            #                     temp = self.c(s, s_) + self.g[s_]
            #                     if min_s > temp:
            #                         min_s = temp
            #                 self.rhs[s] = min_s
            #         self.update_vertex(u)
        
    def myMain(self, canvas, image, grid, map_values):
        """robot by mal vypocitat trasu, vykreslit ju, a urobit jeden krok z naplanovanej trasy"""
        # todo s_last = s_start
        # to do compute shortest path


        if self.positionx != self.goal_pose_x and self.positiony != self.goal_pose_y:  #main while loop
            # check ci existuje cesta 

            self.initialize_dstarlite(map_values)
            self.compute_shortest_path(map_values)


            # self.update_pose(new_x=new_x, new_y=new_y, new_theta=new_theta, canvas=canvas, image=image)

        # path = [robot_position]
        # self.s_start = robot_position
        # self.s_last = self.s_start

        # self.compute_shortest_path()

        # while self.s_start != self.s_goal:
        #     assert (self.rhs[self.s_start] != float('inf')), "There is no known path!"

        #     succ = self.sensed_map.succ(self.s_start, avoid_obstacles=False)
        #     min_s = float('inf')
        #     arg_min = None
        #     for s_ in succ:
        #         temp = self.c(self.s_start, s_) + self.g[s_]
        #         if temp < min_s:
        #             min_s = temp
        #             arg_min = s_


        #     self.s_start = arg_min
        #     path.append(self.s_start)

        #     changed_edges_with_old_cost = self.rescan()

        #     # if any edge costs changed
        #     if changed_edges_with_old_cost:
        #         self.k_m += heuristic(self.s_last, self.s_start)
        #         self.s_last = self.s_start

        #         # for all directed edges (u,v) with changed edge costs
        #         vertices = changed_edges_with_old_cost.vertices
        #         for vertex in vertices:
        #             v = vertex.pos
        #             succ_v = vertex.edges_and_c_old
        #             for u, c_old in succ_v.items():
        #                 c_new = self.c(u, v)
        #                 if c_old > c_new:
        #                     if u != self.s_goal:
        #                         self.rhs[u] = min(self.rhs[u], self.c(u, v) + self.g[v])
        #                 elif self.rhs[u] == c_old + self.g[v]:
        #                     if u != self.s_goal:
        #                         min_s = float('inf')
        #                         succ_u = self.sensed_map.succ(vertex=u)
        #                         for s_ in succ_u:
        #                             temp = self.c(u, s_) + self.g[s_]
        #                             if min_s > temp:
        #                                 min_s = temp
        #                         self.rhs[u] = min_s
        #                     self.update_vertex(u)
        #     self.compute_shortest_path()
        # print("path found!")
        # return path, self.g, self.rhs

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
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





# class MinimalSubscriber(Node):
    

#     def __init__(self):
        
#         super().__init__('minimal_subscriber')
#         self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
#         self.winH = 400
#         self.winW = 400
#         window = Tk()
#         self.canvas = Canvas(window, width=self.winW, height=self.winH, borderwidth=0, highlightthickness=0)

#         self.goal_pose_x = 200
#         self.goal_pose_y = 200


#         self.img_p = PhotoImage(file="/home/bolka/dev_ws/src/pngwing.png")
#         self.positionx = 200
#         self.positiony = 200
#         self.theta = 0
#         self.flag = False
#         self.first_time = True
#         self.L = 1

#     def rotatedPhotoImage(self, img, angle):
#         angleInRads = angle
#         diagonal = sqrt(img.width()**2 + img.height()**2)
#         xmidpoint = img.width()/2
#         ymidpoint = img.height()/2
#         newPhotoImage = PhotoImage(width=int(diagonal), height=int(diagonal))
#         for x in range(img.width()):
#             for y in range(img.height()):

#                 xnew = float(x)
#                 ynew = float(-y)


#                 xnew = xnew - xmidpoint
#                 ynew = ynew + ymidpoint


#                 xnew, ynew = xnew*cos(angleInRads) - ynew*sin(angleInRads), xnew * sin(angleInRads) + ynew*cos(angleInRads)

#                 xnew = xnew + diagonal/2
#                 ynew = ynew - diagonal/2


#                 xnew = xnew
#                 ynew = -ynew

#                 rgb = '#%02x%02x%02x' % img.get(x, y)

#                 newPhotoImage.put(rgb, (int(xnew), int(ynew)))

#                 newPhotoImage.put(rgb, (int(xnew+1), int(ynew)))

#         return newPhotoImage

#     def listener_callback(self, msg):
#         canvas=self.canvas

#         map_values = np.array(literal_eval(msg.data))
#         ncols, nrows= len(map_values[0]), len(map_values)

#         cellW = self.winW / ncols
#         cellH = self.winH / nrows

#         self.side_x = self.winH/ncols
#         self.side_y = self.winW/nrows

       
#         img = self.rotatedPhotoImage(self.img_p, -self.theta)

#         image = canvas.create_image(self.positionx, self.positiony, image=img)
#         canvas.tag_raise(image)
#         canvas.update()


#         class Node:
#             def __init__(self, row, col, val):
#                 self.row = row
#                 self.col = col
#                 self.val = val
#                 return

#         def generatGrid(nrows, ncols):
#             grid = []
#             for r in range(nrows):
#                 row = [ Node(r, c, map_values[r][c]) for c in range(ncols) ]
#                 grid.append(row)
#             return grid

#         def drawNode(canvas, node):
#             x1 = cellW * node.col
#             y1 = cellH * node.row
#             x2 = x1 + cellW
#             y2 = y1 + cellH
#             if node.val==1:
#                 canvas.create_rectangle(x1, y1, x2, y2, fill='gray')
#             else:
#                 canvas.create_rectangle(x1, y1, x2, y2, fill='white')
#             return

#         def drawGrid(canvas, grid):
#             for row in grid:
#                 for node in row:
#                     drawNode(canvas, node)
#             return

#         def left_click(event):
#             y_pos = int(event.x // self.side_x)
#             x_pos = int(event.y // self.side_y)

#             grid = self.grid
#             if grid[x_pos][y_pos].val == 1:
#                 grid[x_pos][y_pos].val  = 0
#                 map_values[x_pos][y_pos] = 0
#             elif grid[x_pos][y_pos].val == 0:
#                 grid[x_pos][y_pos].val  = 1
#                 map_values[x_pos][y_pos] = 1

#             np.savetxt('/home/bolka/dev_ws/src/data.csv', map_values, delimiter=',')

#         def right_click(event):
#             y_pos = int(event.x // self.side_x)
#             x_pos = int(event.y // self.side_y)
#             if self.grid[x_pos][y_pos].val == 0:
#                 self.goal_pose_x = event.x
#                 self.goal_pose_y = event.y

#         canvas.bind("<Button-1>", left_click)
#         canvas.bind("<Button-3>", right_click)

#         self.grid = generatGrid(nrows, ncols)
#         drawGrid(canvas, self.grid)
#         canvas.pack(side = 'top') 
        

#         self.move2goal(canvas, image, self.grid)     


#     def update_pose(self, new_x, new_y, new_theta, canvas, image):
#         self.positionx = new_x
#         self.positiony = new_y
#         self.theta = new_theta

#         print(self.positionx, self.positiony, self.theta)
#         canvas.delete(image)
#         img = PhotoImage(file="/home/bolka/dev_ws/src/pngwing.png")
#         img = self.rotatedPhotoImage(img,-(self.theta))
#         image = canvas.create_image(self.positionx, self.positiony, image=img)
#         canvas.tag_raise(image)
#         canvas.update()
        
 
#     def euclidean_distance(self):
#         return sqrt(pow((self.goal_pose_x - self.positionx), 2) +
#                     pow((self.goal_pose_y - self.positiony), 2))

#     def linear_vel(self, constant=1.2):
#         return constant * self.euclidean_distance()

#     def steering_angle(self, goal_pose_x, goal_pose_y):
#         return atan2(goal_pose_y  - self.positiony, goal_pose_x  - self.positionx)
        
#     def angular_vel(self, constant=12):
#         return constant * (self.steering_angle(self.goal_pose_x, self.goal_pose_y) - self.theta)


#     def check_walls(self, grid, new_x, new_y, new_theta, canvas, image):
#         x_pos = int((new_x) // self.side_x)
#         y_pos = int(new_y // self.side_y)
#         if grid[y_pos][x_pos].val == 1:
#                 self.update_pose(new_x=self.positionx, new_y=self.positiony, new_theta=self.theta+pi, canvas=canvas, image=image)
#                 return True
#         return False

        
#     def move2goal(self, canvas, image, grid):

#         if self.euclidean_distance() > 2:
#             # time.sleep(0.01)

#             v = float(self.linear_vel())
#             w = self.angular_vel()
#             dleft = (v - 1/2*self.L*w) * 0.1
#             dright = (v + 1/2*self.L*w) * 0.1 
#             dcenter = (dleft + dright) / 2
#             phi = (dright - dleft) / self.L

#             delta_x = dcenter * cos(self.theta) 
#             delta_y = dcenter * sin(self.theta)
#             delta_th = self.theta + phi
#             new_x = self.positionx+delta_x
#             new_y = self.positiony+delta_y
#             new_theta = delta_th

#             if self.check_walls(grid, new_x, new_y, new_theta, canvas, image):
#                 v=0
#                 w=0
#                 self.goal_pose_x = self.positionx
#                 self.goal_pose_y = self.positiony
#             self.update_pose(new_x=new_x, new_y=new_y, new_theta=new_theta, canvas=canvas, image=image)

            
# def main(args=None):
#     rclpy.init(args=args)

#     minimal_subscriber = MinimalSubscriber()
#     rclpy.spin(minimal_subscriber)

#     minimal_subscriber.destroy_node()

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

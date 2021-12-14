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

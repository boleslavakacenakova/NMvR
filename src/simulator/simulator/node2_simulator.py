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



class MinimalSubscriber(Node):
    
    # root.mainloop()
    def __init__(self):
        
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.winH = 400
        self.winW = 400
        window = Tk()
        self.canvas = Canvas(window, width=self.winW, height=self.winH, borderwidth=0, highlightthickness=0)
        # # Get the input from the user.

        # Get the input from the user.
        self.goal_pose_x = 200
        self.goal_pose_y = 200


        # self.position= Odometry()
        # self.image = Image.open("/home/bolka/dev_ws/src/pngwing.png")
        self.img_p = PhotoImage(file="/home/bolka/dev_ws/src/pngwing.png")
        self.positionx = 200
        self.positiony = 200
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

                # convert to ordinary mathematical coordinates
                xnew = float(x)
                ynew = float(-y)

                # shift to origin
                xnew = xnew - xmidpoint
                ynew = ynew + ymidpoint

                # new rotated variables, rotated around origin (0,0) using simoultaneous assigment
                xnew, ynew = xnew*cos(angleInRads) - ynew*sin(angleInRads), xnew * sin(angleInRads) + ynew*cos(angleInRads)

                # shift back to quadrant iv (x,-y), but centered in bigger box
                xnew = xnew + diagonal/2
                ynew = ynew - diagonal/2

                # convert to -y coordinates
                xnew = xnew
                ynew = -ynew

                # get pixel data from the pixel being rotated in hex format
                rgb = '#%02x%02x%02x' % img.get(x, y)

                # put that pixel data into the new image
                newPhotoImage.put(rgb, (int(xnew), int(ynew)))

                # this helps fill in empty pixels due to rounding issues
                newPhotoImage.put(rgb, (int(xnew+1), int(ynew)))

        return newPhotoImage

    def listener_callback(self, msg):
        canvas=self.canvas
        #added
        map_values = np.array(literal_eval(msg.data))
        ncols, nrows= len(map_values[0]), len(map_values)

        cellW = self.winW / ncols
        cellH = self.winH / nrows

        self.side_x = self.winH/ncols
        self.side_y = self.winW/nrows

       
        img = self.rotatedPhotoImage(self.img_p, self.theta)

        image = canvas.create_image(self.positionx, self.positiony, image=img)
        canvas.tag_raise(image)
        canvas.update()
        # self.goal_pose_x = float(input("Set your x goal: "))
        # self.goal_pose_y = float(input("Set your y goal: "))


        # self.anglos = float(input("Set your y goal: "))
        # canvas.delete(image)
        # img = PhotoImage(file="/home/bolka/dev_ws/src/pngwing.png")
        # # img = self.rotatedPhotoImage(img,self.theta)
        # img = self.rotatedPhotoImage(img,self.anglos-(pi/2))
        # image = canvas.create_image(self.positionx, self.positiony, image=img)
        # canvas.tag_raise(image)
        # canvas.update()
        # time.sleep(2)


        class Node:
            def __init__(self, row, col, val):
                self.row = row
                self.col = col
                self.val = val
                return

        def generatGrid(nrows, ncols):
            grid = []
            for r in range(nrows):
                row = [ Node(r, c, map_values[r][c]) for c in range(ncols) ]
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

        self.grid = generatGrid(nrows, ncols)
        drawGrid(canvas, self.grid)
        canvas.pack(side = 'top') 
        

        self.move2goal(canvas, image, self.grid)     


    def update_pose(self, new_x, new_y, new_theta, canvas, image):
        self.positionx = new_x
        self.positiony = new_y
        self.theta = new_theta

        print(self.positionx, self.positiony, self.theta)
        canvas.delete(image)
        img = PhotoImage(file="/home/bolka/dev_ws/src/pngwing.png")
        img = self.rotatedPhotoImage(img,(-self.theta))
        # img = self.rotatedPhotoImage(img,self.anglos)
        image = canvas.create_image(self.positionx, self.positiony, image=img)
        canvas.tag_raise(image)
        canvas.update()
        
 
    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((self.goal_pose_x - self.positionx), 2) +
                    pow((self.goal_pose_y - self.positiony), 2))

    def linear_vel(self, constant=1.2):
        return constant * self.euclidean_distance()

    def steering_angle(self, goal_pose_x, goal_pose_y):
        # return (math.remainder(atan2(goal_pose_y  - self.positiony, goal_pose_x  - self.positionx),tau))
        return atan2(goal_pose_y  - self.positiony, goal_pose_x  - self.positionx)
        
    def angular_vel(self, constant=12):
        return constant * (self.steering_angle(self.goal_pose_x, self.goal_pose_y) - self.theta)


    def check_walls(self, grid, new_x, new_y, new_theta, canvas, image):
        x_pos = int((new_x) // self.side_x)
        y_pos = int(new_y // self.side_y)
        if grid[y_pos][x_pos].val == 1:
                self.update_pose(new_x=self.positionx, new_y=self.positiony, new_theta=self.theta+pi, canvas=canvas, image=image)
                return True
        return False

        
    def move2goal(self, canvas, image, grid):

        while self.euclidean_distance() > 2:
            time.sleep(0.01)



            v = float(self.linear_vel())
            w = self.angular_vel()
            dleft = (v - 1/2*self.L*w) * 0.1
            dright = (v + 1/2*self.L*w) * 0.1 
            dcenter = (dleft + dright) / 2
            phi = (dright - dleft) / self.L

            delta_x = dcenter * cos(self.theta) 
            delta_y = dcenter * sin(self.theta)
            # delta_th = (math.remainder(self.theta+ phi,tau))
            delta_th = self.theta + phi
            new_x = self.positionx+delta_x
            new_y = self.positiony+delta_y
            new_theta = delta_th

            if self.check_walls(grid, new_x, new_y, new_theta, canvas, image):
                print(new_x,new_y)
                print("akt pozicia ",self.positionx, self.positiony)
                v=0
                w=0
                self.goal_pose_x = self.positionx
                self.goal_pose_y = self.positiony
                break
            self.update_pose(new_x=new_x, new_y=new_y, new_theta=new_theta, canvas=canvas, image=image)

            
        # Stopping our robot after the movement is over.
        v=0
        w=0


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# import numpy as np
# from ast import literal_eval
# from rclpy.node import Node
# from tkinter import *
# from std_msgs.msg import String
# from PIL import Image, ImageTk
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# import sys
# import math
# from math import pow, atan2, sqrt, sin, cos, remainder, tau

# root = Tk()
# class MinimalSubscriber(Node):
    
#     # root.mainloop()
#     def __init__(self):
        
#         super().__init__('minimal_subscriber')
#         self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
#         # self.subscription

#         # self.velocity_publisher = self.create_publisher(Twist,'vel',10)
#         # self.pose_subscriber = self.create_subscription(Odometry,'odom', self.update_pose, 10)

#         # # Get the input from the user.
#         # self.goal_pose_x = float(input("Set your x goal: "))
#         # self.goal_pose_y = float(input("Set your y goal: "))
#                 # Get the input from the user.
#         self.goal_pose_x = 3
#         self.goal_pose_y = 3


#         # self.position= Odometry()
#         self.image = Image.open("/home/bolka/dev_ws/src/pngwing.png")
#         self.positionx = 1
#         self.positiony = 1
#         self.theta = 0
#         self.flag = False
#         self.first_time = True
#         self.L = 1

#     def listener_callback(self, msg):
#         button_list = ['dummy']
#         # root = Tk()
#         # root.columnconfigure(0, minsize=8)
#         map_values = np.array(literal_eval(msg.data))
#         size_x, size_y = len(map_values[0]), len(map_values)
#         button_number = 1

#         class button_box:
#             def __init__(self, button, ID_number):
#                 self.ID_number = ID_number
#                 self.button = button

#             def clicked(self, event):
#                 if self.button.info['val'] == 1:
#                     self.button.config(bg='white')
#                     self.button.info['val'] = 0
#                 elif self.button.info['val'] == 0:
#                     self.button.info['val'] = 1
#                     self.button.config(bg='gray')
#                 print(self.button.info)
#                 x = self.button.info['x']
#                 y = self.button.info['y']
#                 val = self.button.info['val']
#                 map_values[y][x] = val
#                 print(x, y, val)
#                 np.savetxt('/home/bolka/dev_ws/src/data.csv', map_values, delimiter=',')

#         if self.first_time:
#             for y in range(size_y):
#                 for x in range(size_x):
#                     button = Button(width=8, height=4)
#                     button.config(relief='solid', borderwidth=1)
#                     if map_values[y][x] == 0:
#                         button.config(bg='white')
#                     elif map_values[y][x] == 1:
#                         button.config(bg='gray')
#                     button.grid(row=y, column=x)
#                     button.info = {"val": map_values[y][x],
#                                 "x": x,
#                                 "y": y}

#                     button_list.append(button_box(button, button_number))
#                     button.bind('<Button-1>', button_list[button_number].clicked)
#                     button_number += 1

#             self.photo = ImageTk.PhotoImage(self.image.rotate(self.theta))
#             self.robot = Label(root, image=self.photo)
#             self.robot.image = self.photo
#             self.robot.grid(row=5, column=5)

#             self.first_time = False

#         self.move2goal()
#         # self.update_pose(new_x=self.positionx+1, new_y=self.positiony+1, new_theta=self.theta+10)
#         # self.robot.grid(row=round(self.goal_pose_x), column=round(self.goal_pose_y))
#         # self.get_logger().info("ahoj")
#         root.update()


#     def update_pose(self, new_x, new_y, new_theta):
#         self.positionx = new_x
#         self.positiony = new_y
#         self.theta = new_theta
#         print(self.positionx, self.positiony, self.theta)

#         self.robot.grid(row=round(self.positionx), column=round(self.positiony))
#         # self.canvas.move(self.image, round(self.positionx), round(self.positiony))

 
#     def euclidean_distance(self):
#         """Euclidean distance between current pose and the goal."""
#         return sqrt(pow((self.goal_pose_x - self.positionx), 2) +
#                     pow((self.goal_pose_y - self.positiony), 2))

#     def linear_vel(self, constant=1.5):
#         return constant * self.euclidean_distance()

#     def steering_angle(self, goal_pose_x, goal_pose_y):
#         return (math.remainder(atan2(goal_pose_y  - self.positiony, goal_pose_y  - self.positionx), tau))

#     def angular_vel(self, constant=6):
#         return constant * (self.steering_angle(self.goal_pose_x, self.goal_pose_y) - self.theta)

#     def move2goal(self):
#         vel_msg = Twist()

#         # while self.euclidean_distance() >= 0.1:
#         if self.euclidean_distance() >= 1:
#             print(self.euclidean_distance())

#             # Linear velocity in the x-axis.
#             v = float(self.linear_vel())
#             # vel_msg.linear.y = float(0)
#             # vel_msg.linear.z = float(0)

#             # Angular velocity in the z-axis.
#             # vel_msg.angular.x = float(0)
#             # vel_msg.angular.y = float(0)
#             w = self.angular_vel()

#             # Publishing our vel_msg
#             # self.velocity_publisher.publish(vel_msg)

#             # Publish at the desired rate.
#             # self.rate.sleep()
#             dleft = (v + 1/2*self.L*w) * 0.1
#             dright = (v - 1/2*self.L*w) * 0.1 
#             dcenter = (dleft + dright) / 2
#             phi = (dright - dleft) / self.L
#             delta_x = (dcenter * cos(self.theta)) 
#             delta_y = (dcenter * sin(self.theta)) 
#             delta_th = phi

#             self.update_pose(new_x=self.positionx+delta_x, new_y=self.positiony+delta_y, new_theta=self.theta + delta_th)
#         # Stopping our robot after the movement is over.
#         else:
#             v=0
#             vth=0
#         # self.velocity_publisher.publish(vel_msg)

        

# def main(args=None):
#     rclpy.init(args=args)

#     minimal_subscriber = MinimalSubscriber()
#     # minimal_subscriber.move2goal()
#     rclpy.spin(minimal_subscriber)

#     minimal_subscriber.destroy_node()

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

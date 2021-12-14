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
from math import pow, atan2, sqrt, sin, cos

root = Tk()
class MinimalSubscriber(Node):
    
    # root.mainloop()
    def __init__(self):
        
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        # self.subscription

        # self.velocity_publisher = self.create_publisher(Twist,'vel',10)
        # self.pose_subscriber = self.create_subscription(Odometry,'odom', self.update_pose, 10)

        # # Get the input from the user.
        # self.goal_pose_x = float(input("Set your x goal: "))
        # self.goal_pose_y = float(input("Set your y goal: "))
                # Get the input from the user.
        self.goal_pose_x = 3
        self.goal_pose_y = 3


        # self.position= Odometry()
        self.image = Image.open("/home/bolka/dev_ws/src/pngwing.png")
        self.positionx = 1
        self.positiony = 1
        self.theta = 0
        self.flag = False
        self.first_time = True

    def listener_callback(self, msg):
        button_list = ['dummy']
        # root = Tk()
        # root.columnconfigure(0, minsize=8)
        map_values = np.array(literal_eval(msg.data))
        size_x, size_y = len(map_values[0]), len(map_values)
        button_number = 1

        class button_box:
            def __init__(self, button, ID_number):
                self.ID_number = ID_number
                self.button = button

            def clicked(self, event):
                if self.button.info['val'] == 1:
                    self.button.config(bg='white')
                    self.button.info['val'] = 0
                elif self.button.info['val'] == 0:
                    self.button.info['val'] = 1
                    self.button.config(bg='gray')
                print(self.button.info)
                x = self.button.info['x']
                y = self.button.info['y']
                val = self.button.info['val']
                map_values[y][x] = val
                print(x, y, val)
                np.savetxt('/home/bolka/dev_ws/src/data.csv', map_values, delimiter=',')

        if self.first_time:
            for y in range(size_y):
                for x in range(size_x):
                    button = Button(width=8, height=4)
                    button.config(relief='solid', borderwidth=1)
                    if map_values[y][x] == 0:
                        button.config(bg='white')
                    elif map_values[y][x] == 1:
                        button.config(bg='gray')
                    button.grid(row=y, column=x)
                    button.info = {"val": map_values[y][x],
                                "x": x,
                                "y": y}

                    button_list.append(button_box(button, button_number))
                    button.bind('<Button-1>', button_list[button_number].clicked)
                    button_number += 1
            self.photo = ImageTk.PhotoImage(self.image.rotate(self.theta))
            self.robot = Label(root, image=self.photo)
            self.robot.image = self.photo
            self.get_logger().info("iba raz")
            self.robot.grid(row=0, column=0)
            self.first_time = False

        self.move2goal()
        # self.update_pose(new_x=self.positionx+1, new_y=self.positiony+1, new_theta=self.theta+10)
        # self.robot.grid(row=round(self.goal_pose_x), column=round(self.goal_pose_y))
        self.get_logger().info("ahoj")
        root.update()


    def update_pose(self, new_x, new_y, new_theta):
        self.positionx = new_x
        self.positiony = new_y
        self.theta = new_theta
        print(self.positionx, self.positiony, self.theta)
        # self.image = Image.open("/home/bolka/dev_ws/src/pngwing.png")

        # self.photo = self.image.rotate(self.theta)
        # self.robot = Label(root, image=self.photo)
        # self.robot.image = self.photo
        self.robot.grid(row=round(self.positionx), column=round(self.positiony))

 
    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((self.goal_pose_x - self.positionx), 2) +
                    pow((self.goal_pose_y - self.positiony), 2))

    def linear_vel(self, constant=1.5):
        return constant * self.euclidean_distance()

    def steering_angle(self, goal_pose_x, goal_pose_y):
        return atan2(goal_pose_y  - self.positiony, goal_pose_y  - self.positionx)

    def angular_vel(self, constant=6):
        return constant * (self.steering_angle(self.goal_pose_x, self.goal_pose_y) - self.theta)

    def move2goal(self):
        vel_msg = Twist()

        # while self.euclidean_distance() >= 0.1:
        if self.euclidean_distance() >= 1:

            # Linear velocity in the x-axis.
            v = float(self.linear_vel())
            # vel_msg.linear.y = float(0)
            # vel_msg.linear.z = float(0)

            # Angular velocity in the z-axis.
            # vel_msg.angular.x = float(0)
            # vel_msg.angular.y = float(0)
            vth = self.angular_vel()

            # Publishing our vel_msg
            # self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            # self.rate.sleep()

        # Stopping our robot after the movement is over.
        else:
            v=0
            vth=0
        # self.velocity_publisher.publish(vel_msg)
        delta_x = (v * cos(self.theta)) 
        delta_y = (v * sin(self.theta)) 
        delta_th = vth 

        print(v,vth)
        print(delta_x,delta_y)
        print("euc dis %d", self.euclidean_distance())
        self.update_pose(new_x=self.positionx+delta_x, new_y=self.positiony+delta_y, new_theta=self.theta)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    # minimal_subscriber.move2goal()
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
# from math import pow, atan2, sqrt

# root = Tk()
# class MinimalSubscriber(Node):
    
#     # root.mainloop()
#     def __init__(self):
        
#         super().__init__('minimal_subscriber')
#         self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
#         # self.subscription

#         # self.velocity_publisher = self.create_publisher(Twist,'vel',10)
#         # self.pose_subscriber = self.create_subscription(Odometry,'odom', self.update_pose, 10)

#         # Get the input from the user.
#         self.goal_pose_x = float(input("Set your x goal: "))
#         self.goal_pose_y = float(input("Set your y goal: "))


#         self.position= Odometry()
#         self.positionx = 0
#         self.positiony = 0
#         self.yaw = 0
#         self.flag = False
#         self.first_time = True

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
#             image = Image.open("/home/bolka/dev_ws/src/pngwing.png")
#             photo = ImageTk.PhotoImage(image)
#             self.robot = Label(root, image=photo)
#             self.robot.image = photo
#             self.get_logger().info("iba raz")
#             self.first_time = False

#         self.move2goal()
#         self.robot.grid(row=round(self.goal_pose_x), column=round(self.goal_pose_y))
#         self.get_logger().info("ahoj")
#         root.update()


#     def update_pose(self, msg):
#         self.positionx = msg.pose.pose.position.x
#         self.positiony = msg.pose.pose.position.y
#         self.orientation_q = msg.pose.pose.orientation
#         self.orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
#         (self.roll, self.pitch, self.yaw) = euler_from_quaternion (self.orientation_list)
#         print(self.positionx, self.positiony, self.yaw)
#         robot.grid(row=round(self.robot_pose.x, 4), column=round(self.robor_pose.y, 4))
 
#     def euclidean_distance(self):
#         """Euclidean distance between current pose and the goal."""
#         return sqrt(pow((self.goal_pose_x - self.positionx), 2) +
#                     pow((self.goal_pose_y - self.positiony), 2))

#     def linear_vel(self, constant=1.5):
#         return constant * self.euclidean_distance()

#     def steering_angle(self, goal_pose_x, goal_pose_y):
#         return atan2(goal_pose_y  - self.positiony, goal_pose_y  - self.positionx)

#     def angular_vel(self, constant=6):
#         return constant * (self.steering_angle(self.goal_pose_x, self.goal_pose_y) - self.yaw)

#     def move2goal(self):
#         vel_msg = Twist()

#         while self.euclidean_distance() >= 0.1:

#             # Linear velocity in the x-axis.
#             vel_msg.linear.x = float(self.linear_vel())
#             vel_msg.linear.y = float(0)
#             vel_msg.linear.z = float(0)

#             # Angular velocity in the z-axis.
#             vel_msg.angular.x = float(0)
#             vel_msg.angular.y = float(0)
#             vel_msg.angular.z = self.angular_vel()

#             # Publishing our vel_msg
#             self.velocity_publisher.publish(vel_msg)

#             # Publish at the desired rate.
#             # self.rate.sleep()
  
#         # Stopping our robot after the movement is over.
#         vel_msg.linear.x = 0
#         vel_msg.angular.z = 0
#         self.velocity_publisher.publish(vel_msg)


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_subscriber = MinimalSubscriber()
#     # minimal_subscriber.move2goal()
#     rclpy.spin(minimal_subscriber)

#     minimal_subscriber.destroy_node()

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# import rclpy
# import numpy as np
# from ast import literal_eval
# from rclpy.node import Node
# from tkinter import *
# from std_msgs.msg import String
# from PIL import Image, ImageTk

# class MinimalSubscriber(Node):
#     def __init__(self):
#         super().__init__('minimal_subscriber')
#         self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
#         self.subscription

#     def listener_callback(self, msg):
#         button_list = ['dummy']
#         root = Tk()
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


#         for y in range(size_y):
#             for x in range(size_x):
#                 button = Button(width=8, height=4)
#                 button.config(relief='solid', borderwidth=1)
#                 if map_values[y][x] == 0:
#                     button.config(bg='white')
#                 elif map_values[y][x] == 1:
#                     button.config(bg='gray')
#                 button.grid(row=y, column=x)
#                 button.info = {"val": map_values[y][x],
#                             "x": x,
#                             "y": y}

#                 button_list.append(button_box(button, button_number))
#                 button.bind('<Button-1>', button_list[button_number].clicked)
#                 button_number += 1
#         image = Image.open("/home/bolka/dev_ws/src/pngwing.png")
#         photo = ImageTk.PhotoImage(image)
#         label = Label(root, image=photo)
#         label.image = photo
#         label.grid(row=5, column=5)
#         root.mainloop()

# def main(args=None):
#     rclpy.init(args=args)

#     minimal_subscriber = MinimalSubscriber()

#     rclpy.spin(minimal_subscriber)

#     minimal_subscriber.destroy_node()

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
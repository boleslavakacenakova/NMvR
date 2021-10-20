import rclpy
import numpy as np
from ast import literal_eval
from rclpy.node import Node
from tkinter import *
from std_msgs.msg import String
from PIL import Image, ImageTk

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        button_list = ['dummy']
        root = Tk()

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
                np.savetxt('/home/bolka/Documents/nmvr/zadanie_1/data.csv', map_values, delimiter=',')


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
        image = Image.open("/home/bolka/Documents/nmvr/zadanie_1/pngwing.png")
        photo = ImageTk.PhotoImage(image)
        label = Label(root, image=photo)
        label.image = photo
        label.grid(row=1)
        root.mainloop()

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
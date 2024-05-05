import math
import serial
import threading
import time

zero = [0, 0, 0]
end_coords = [0, 0, 0,
              0, 0, 0]


class Link:
    def __init__(self, name: str, length: float, pivot_axis: str, normal: list, ):
        self.name = name
        self.length = length
        self.pivot_axis = pivot_axis
        self.normal = normal

        match self.pivot_axis:
            case "Pitch":
                self.pivot_axis = 3
            case "Roll":
                self.pivot_axis = 4
            case "Yaw":
                self.pivot_axis = 5
            case _:
                print(f"pivot access on link {self.name} fail!")
                return

    def calc_coords(self, initial_coords: list, angle: float):  # x y z - pitch roll yaw
        output_coords = initial_coords
        output_coords[self.pivot_axis] += angle

        output_normal = self.normal
        for x in range(2):
            output_normal[x] += output_coords[x+3]

        xy_magnitude = math.cos(output_coords[3])*self.length
        xy_magnitude_normal = math.cos(output_normal[0])*output_normal[3]

        output_coords[0] += math.cos(output_coords[5])*xy_magnitude + math.cos(output_normal[2]) * xy_magnitude_normal
        output_coords[1] += math.sin(output_coords[5])*xy_magnitude + math.sin(output_normal[2]) * xy_magnitude_normal
        output_coords[2] += math.sin(output_coords[3])*self.length + math.sin(output_normal[0]) * output_normal[3]

        print(output_normal)
        return output_coords


Base = Link("nya", 255.49, "Pitch", [0, 0, -math.pi/2, 112.15])
print(Base.calc_coords([0, 0, 0, 0, 0, 0],
                       math.radians(45)))

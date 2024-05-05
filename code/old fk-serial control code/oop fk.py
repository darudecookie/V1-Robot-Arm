import math
import serial
import threading
import time

zero = [0, 0, 0]
end_coords = [0, 0, 227.075]
end_rot = [0, 0, 0]


class ArmSection:
    def __init__(self, name: str, distance: float,  angletype: str,   normal: tuple, locked=False):
        self.name = name
        self.distance = distance
        self.angletype = angletype
        #self.const_angle = const_angle
        #Link.limits = limits
        self.locked = locked
        self.normal = normal

        match self.angletype:
            case "Pitch":
                self.angletype = 0
            case "Roll":
                self.angletype = 1
            case "Yaw":
                self.angletype = 2


    def meow(self, initial_pos: list, initial_rot: list, angle: list):

        final_pos = initial_pos
        final_rot = initial_rot
        final_rot[self.angletype] += angle

        final_normal=self.normal
        for x in range(2):
            final_normal[x] += final_rot[x]

        print(final_normal)


        xy_magnitude = self.distance*math.cos(final_rot[0])
        xy_magnitude_normal = self.normal[3]*math.cos(0)
        final_pos[0] = xy_magnitude * \
            math.cos(final_rot[2])+xy_magnitude_normal * \
            math.cos(final_normal[2])
        final_pos[1] = xy_magnitude * \
            math.sin(final_rot[2])+xy_magnitude_normal * \
            math. sin(final_normal[2])

        final_pos[2] += self.distance * \
            math.sin(final_rot[0]) +final_normal[3]*math.cos(final_normal[0])

        return final_pos, final_rot

    def toggle(self, key: bool):
        self.locked = key


Link1_2 = ArmSection("Base (Axis 1-2)", 0, "Yaw",
                     [math.pi/2, 0, 0, 0])
Link2_3 = ArmSection("Big Link (Axis 2-3)", 255.49, "Pitch",
                     [0, 0, -math.pi/2, 112.5])

one = math.radians(15)
two = math.radians(45)

end_coords, end_rot = Link1_2.meow(end_coords, end_rot, one)
end_coords, end_rot = Link2_3.meow(end_coords, end_rot, two)

for x in range(3):
    end_coords[x] = round(end_coords[x])
    end_rot[x] = round(math.degrees(end_rot[x]))

print(end_coords, end_rot)


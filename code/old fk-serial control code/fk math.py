import math
import serial
import time
import threading


ser = serial.Serial("COM4", 9600)


height = 227.075
arm2_3 = 255.5
arm3_4 = 117.54
arm3_4_horizontal = -112.15


angles = [0, 0, 0, 0]  # angles of axis 1-4 in degrees


def make_normal(x):
    x = math.radians(x)
    while x >= 2*math.pi:
        x -= 2*math.pi
    while x < 2*math.pi:
        x += 2*math.pi
    return x


def make_normal_deg(x):
    x = math.degrees(x)
    while x >= 360:
        x -= 360
    while x < 0:
        x += 360
    return x


def mather(angles):
    end_pose = [[0, 0, 0],  # x / y / z
                [0, 0, 0]]  # pitch / roll / yaw

    absA3 = (angles[1] + angles[2])
    absA3 = make_normal(absA3)

    for i in range(len(angles)):
        angles[i] = make_normal(angles[i])

    xy_len_arm2_3 = arm2_3*math.cos(angles[1])
    xy_len_arm3_4 = arm3_4*math.cos(absA3)

    end_pose[0][0] = (xy_len_arm2_3+xy_len_arm3_4)*math.sin(angles[0]
                                                            ) - arm3_4_horizontal*math.sin(angles[0]+math.pi/2)

    end_pose[0][1] = (xy_len_arm2_3+xy_len_arm3_4)*math.cos(angles[0]
                                                            ) - arm3_4_horizontal*math.cos(angles[0]+math.pi/2)

    end_pose[0][2] = height + arm2_3 * \
        math.sin(angles[1]) + arm3_4*math.sin(absA3)

    end_pose[1][0] = absA3
    end_pose[1][1] = angles[0]
    end_pose[1][2] = angles[3]

    for i in range(3):
        end_pose[0][i] = round(end_pose[0][i], 2)
        end_pose[1][i] = make_normal_deg(end_pose[1][i])
        end_pose[1][i] = round(end_pose[1][i], 2)

    return end_pose


def sender():
    while True:
        angles = [0, 0, 0, 0]
        for i in range(4):
            angles[i] = float(input(f"axis {i+1} angle? "))
        print(
            f"those angles would move to {mather(angles)} -- (( [x, y, z], [pitch, roll, yaw,] ))")

        for i in range(len(angles)):
            angles[i] = make_normal_deg(angles[i])

        data = f"{angles[0]}, {angles[1]}, {angles[2]}, {angles[3]}"
        data = data.encode()
        print(data)

        if input("send coords? (y/n)") == "y":
            ser.write(data)
            print("sending")
            if ser.read() == "data error!":
                print("data error!")

instructions = "0,0,0,20,-90,-90\n"

ser.write(bytes(instructions,'utf-8'))
ser.close()


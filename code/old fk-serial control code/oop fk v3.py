import math
import time
import numpy

start=time.time()
angles=[0,0,0,0,0,0]
#theta, alpha, r/a, d

joint_dh_params=[
[angles[0], 90, 0, 217.75],
[angles[1], 180, 255.5, 0],
[angles[2], -90, 0, 97.75],
[angles[3], 90, 0, 123.8],
[angles[4], 90, 0, -70],
[angles[5], 0, 5, 22.9],
]


class Arm_Matrix:
    def __init__(self,name,dh):
        self.name=name
        self.dh=dh
        self.dh[0]=math.radians(dh[0])
        self.dh[1]=math.radians(dh[1])
        self.transform_matrix=[
            [math.cos(0),      -math.sin(dh[0])* math.cos(dh[1]),    math.sin(dh[0])*math.sin(dh[1]),    dh[2]* math.cos(dh[0])],
            [math.sin(dh[0]),       math.cos(dh[0])* math.cos(dh[1]),       -math.cos(dh[0])*math.sin(dh[1]),      dh[2]* math.sin(dh[0])],
            [0,     math.sin(dh[1]),    math.cos(dh[1]),     dh[3]],
            [0,     0,      0,      1],
        ]
    def matrixout(self):
        return self.transform_matrix
    def __str__(self):
        return f"{self.transform_matrix[0]}\n{self.transform_matrix[1]}\n{self.transform_matrix[2]}\n{self.transform_matrix[3]}\n"
    def __mul__(self, other):
        matrix_out=self.transform_matrix
        match str(type(other)):
            case "list" :
                for i in range(4):
                    for x in range(4):
                        for w in range(4):
                            matrix_out[i][x] += self.transform_matrix[i][w] * other[w][x]

            case "Arm_Matrix" :
                mult_matrix=other.matrixout()
                for i in range(4):
                    for x in range(4):
                        for w in range(4):
                            matrix_out[i][x] += self.transform_matrix[i][w] * mult_matrix[w][x]
            case _ :
                for i in range(4):
                    for x in range(4):
                        matrix_out[i][x]*=other

            
        return matrix_out


axis0=Arm_Matrix("Base", joint_dh_params[0])
axis1=Arm_Matrix("Link 2-3 (Big Link)", joint_dh_params[1])
axis2=Arm_Matrix("Link 3-4 (Short Link)", joint_dh_params[2])
axis3=Arm_Matrix("Link 3-4 (Rotary Axis)", joint_dh_params[3])
axis4=Arm_Matrix("Link 4-5 (Servo 1 Axis)", joint_dh_params[4])
axis5=Arm_Matrix("Link 5-6 (Servo 2 Axis)", joint_dh_params[5])

Arm_List=[axis0,axis1,axis2,axis3,axis4,axis5]

mreow=axis0*axis1
mreow=axis2*mreow
mreow=axis3*mreow
mreow=axis4*mreow   
mreow=axis5*mreow

for i in range(4):
      print(mreow[i])

print(time.time()-start)
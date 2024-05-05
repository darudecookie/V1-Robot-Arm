import time

start = time.time()
import math
import numpy as np
import random
import threading

import ikpy.chain
import ikpy.utils.plot as plot_utils


angles = [0, 0, 0, 0, 0, 0]


class Arm_Link:
    def __init__(self, name, dh, params=[]):
        self.name = name
        self.dh = dh
        self.dh[0] = math.radians(dh[0])
        self.dh[1] = math.radians(dh[1])
        self.params = params
        self.transform_matrix=[
            [ math.cos(self.dh[0]) , -math.sin(self.dh[0]) * math.cos(self.dh[1]) ,  math.sin(self.dh[0]) * math.sin(self.dh[1]) , self.dh[2]* math.cos(self.dh[0]) ],
            [ math.sin(self.dh[0]) ,  math.cos(self.dh[0]) * math.cos(self.dh[1]) , -math.cos(self.dh[0]) * math.sin(self.dh[1]) , self.dh[2]* math.sin(self.dh[0]) ],
            [                    0 ,                         math.sin(self.dh[1]) ,                         math.cos(self.dh[1]) ,                       self.dh[3] ],
            [                    0 ,                                            0 ,                                             0 ,                               1 ],
        ]

    def matrixout(self,angleinput):
        new_angle = self.dh[0] + math.radians(angleinput)
        self.transform_matrix=[  
            [ math.cos(new_angle) , -math.sin(new_angle) * math.cos(self.dh[1]) ,  math.sin(new_angle) * math.sin(self.dh[1]) , self.dh[2]* math.cos(new_angle) ],
            [ math.sin(new_angle) ,  math.cos(new_angle) * math.cos(self.dh[1]) , -math.cos(new_angle) * math.sin(self.dh[1]) , self.dh[2]* math.sin(new_angle) ],
            [                    0 ,                         math.sin(self.dh[1]) ,                         math.cos(self.dh[1]) ,                       self.dh[3] ],
            [                    0 ,                                            0 ,                                             0 ,                               1 ]
        ]
        return self.transform_matrix

    def __str__(self):
        return f"{self.transform_matrix[0]}\n{self.transform_matrix[1]}\n{self.transform_matrix[2]}\n{self.transform_matrix[3]}\n"


def multiply_by(high, low):
    matrix_output = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ]
    try:
        high.matrixout()
        high_matrix = high.matrixout()
    except AttributeError:
        high_matrix = high
    try:
        low.matrixout()
        low_matrix = low.matrixout()
    except AttributeError:
        low_matrix = low

    for i in range(4):
        for j in range(4):
            for k in range(4):
                matrix_output[i][j] += high_matrix[i][k] * low_matrix[k][j]
    return matrix_output

def magnitude_calc(vector1,vector2):
    output=0
    for i in range(len(vector1)):
        output+=(vector1[i]-vector2[i])**2
    return math.sqrt(output)

def square_magnitude_calc(vector1,vector2):
    output=0
    for i in range(len(vector1)):
        output+=(vector1[i]-vector2[i])**2
    return output


def fk_recalc(ArmList, recalc_angles):
    resultant_matrix = multiply_by(
        ArmList[1].matrixout(recalc_angles[1]), ArmList[0].matrixout(recalc_angles[0])
    )
    for i in range(2, len(ArmList)):
        resultant_matrix = multiply_by(
            ArmList[i].matrixout(recalc_angles[i]), resultant_matrix
        )
    return resultant_matrix


def random_fk_calc(ArmList,target,iterations=1000000):

    output_angles=[]
    output_magnitudes=[]
    
    for i in range(iterations):
        trial_angles=[]
        for x in range(len(ArmList)):
            trial_angles.append(random.randint(joint_params[x][0],joint_params[x][1]))


        trial_matrix_magnitude=fk_recalc(ArmList,trial_angles)
        trial_matrix_magnitude=square_magnitude_calc(target, (trial_matrix_magnitude[0][3],trial_matrix_magnitude[1][3],trial_matrix_magnitude[2][3],))

        output_magnitudes.append(trial_matrix_magnitude)
        output_angles.append((trial_angles))

    index=output_magnitudes.index(min(output_magnitudes))
    return math.sqrt(min(output_magnitudes)), output_angles[index]




joint_dh_params=[
#     theta , alpha ,   r/a , d
    [    90 ,    90 ,     0 , 217.75 ],
    [     0 ,   180 , 255.5 , 0      ],
    [   -90 ,   -90 ,     0 , 97.75  ],
    [     0 ,    90 ,     0 , 123.8  ],
    [    90 ,    90 ,     0 , -70    ],
    [     0 ,     0 ,     5 , 22.9   ],
    ]

joint_params=[
#    low limit, high limit, 
    [-20,200,],
    [-180,180],
    [-180,180],
    [-180,180],
    [-180,180],
    [-180,180],
]

Arm = [
    Arm_Link("Base", joint_dh_params[0]),
    Arm_Link("Link 2-3 (Big Link)", joint_dh_params[1]),
    Arm_Link("Link 3-4 (Short Link)", joint_dh_params[2]),
    Arm_Link("Link 3-4 ( Rotary Axis)", joint_dh_params[3]),
    Arm_Link("Link 4-5 ( Servo 1 Axis)", joint_dh_params[4]),
    Arm_Link("Link 5-6 (Servo 2 Axis)", joint_dh_params[5])
]

start = time.time()
target = (0,0,342)

nya,mreow=(random_fk_calc(Arm, target,10000))

bwuh=fk_recalc(Arm,mreow)

for x in range(4):
    for i in range(4):
        bwuh[x][i]=round(bwuh[x][i],2)
print(bwuh[0])
print(bwuh[1])
print(bwuh[2])
print(bwuh[3])
magnitudes=magnitude_calc(target,(bwuh[0][3],bwuh[1][3],bwuh[2][3]))
print(f"the joint angles {mreow} got us {round(magnitudes,3)} mm away from the target of {target} (x, y ,z) after ")

print(f"that took {round(time.time()-start,4)} seconds to run")
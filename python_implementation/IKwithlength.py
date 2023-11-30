import numpy as np

# radian/degree conversion constants
RD = 57.29578
DR = 0.0174533

# Arm class to create multiple instances of arms with different link lengths
class Arm:
    # Setting length, endeffector, wristeffector, and joint angle attributes
    def __init__(self, lengths):
        self.lengths = lengths
        self.endeffector = np.zeros((4, 4))
        self.wristeffector = np.zeros(4)
        self.angles = np.zeros(6)
    
    # Helper Funcitons for inverse kinematics:
    # Conversion from Euler angles to rotation matrix
    def _convertToEndEffector(self, pos, euler):
        rollMat = np.array([[1, 0, 0],
                           [0, np.cos(euler[0]), -np.sin(euler[0])],
                           [0, np.sin(euler[0]), np.cos(euler[0])]])
        pitchMat = np.array([[np.cos(euler[1]), 0, np.sin(euler[1])],
                            [0, 1, 0],
                            [-np.sin(euler[1]), 0, np.cos(euler[1])]])
        yawMat = np.array([[np.cos(euler[2]), -np.sin(euler[2]), 0],
                          [np.sin(euler[2]), np.cos(euler[2]), 0],
                          [0, 0, 1]])
        rotMat = np.dot(rollMat, np.dot(pitchMat, yawMat))
        self.endeffector = np.array([[rotMat[0, 0], rotMat[0, 1], rotMat[0, 2], pos[0]],
                                    [rotMat[1, 0], rotMat[1, 1], rotMat[1, 2], pos[1]],
                                    [rotMat[2, 0], rotMat[2, 1], rotMat[2, 2], pos[2]],
                                    [0, 0, 0, 1]])
    
    # Determining wrist effetor from endeffector
    def _wristEffector(self, endeffector, link):
        self.wristeffector[0] = endeffector[0, 3] - link[6] * endeffector[0, 2]
        self.wristeffector[1] = endeffector[1, 3] - link[6] * endeffector[1, 2]
        self.wristeffector[2] = endeffector[2, 3] - link[6] * endeffector[2, 2]
        self.wristeffector[3] = endeffector[3, 3] - link[6] * endeffector[3, 2]
    
    # Calculating joint angle 1
    def _angle1(self, wristeffector):
        angle1Result = np.zeros(2)
        angle1Result[0] = np.arctan(wristeffector[1] / wristeffector[0])
        angle1Result[1] = np.arctan(wristeffector[1] / wristeffector[0]) + np.pi
        return angle1Result

    # Calculating joint angle 2
    def _angle2(self, wristeffector, angle1, link):
        angle2Result = np.zeros(4)
        A1 = -(-(wristeffector[2] - (link[1] + link[0])) * link[2] * 2)
        B1 = -(-wristeffector[0] / np.cos(angle1[0]) * link[2] * 2)
        C1 = -(pow(link[5] + link[4], 2) + pow(link[3], 2) - pow(wristeffector[0] / np.cos(angle1[0]), 2) - pow(wristeffector[2] - (link[1] + link[0]), 2) - pow(link[2], 2))

        A2 = -(-(wristeffector[2] - (link[1] + link[0])) * link[2] * 2)
        B2 = -(-wristeffector[0] / np.cos(angle1[1]) * link[2] * 2)
        C2 = -(pow(link[5] + link[4], 2) + pow(link[3], 2) - pow(wristeffector[0] / np.cos(angle1[1]), 2) - pow(wristeffector[2] - (link[1] + link[0]), 2) - pow(link[2], 2))

        angle2Result[0] = np.arctan2(C1 , np.sqrt(pow(A1, 2) + pow(B1, 2) - pow(C1, 2))) - np.arctan2(A1, B1)
        angle2Result[1] = np.arctan2(C1 , -np.sqrt(pow(A1, 2) + pow(B1, 2) - pow(C1, 2))) - np.arctan2(A1, B1)
        angle2Result[2] = np.arctan2(C2 , np.sqrt(pow(A2, 2) + pow(B2, 2) - pow(C2, 2))) - np.arctan2(A2, B2)
        angle2Result[3] = np.arctan2(C2 , -np.sqrt(pow(A2, 2) + pow(B2, 2) - pow(C2, 2))) - np.arctan2(A2, B2)
        return angle2Result
    
    # Calculating joint angle 3
    def _angle3(self, wristeffector, angle1, angle2, link):
        angle3Result = np.zeros(4)
        A = link[4] + link[5]
        B = link[3]
        C1 = wristeffector[0]/np.cos(angle1[0]) - link[2] * np.sin(angle2[0])
        C2 = wristeffector[0]/np.cos(angle1[0]) - link[2] * np.sin(angle2[1])
        C3 = wristeffector[0]/np.cos(angle1[1]) - link[2] * np.sin(angle2[2])
        C4 = wristeffector[0]/np.cos(angle1[1]) - link[2] * np.sin(angle2[3])

        angle3Result[0] = np.arctan2(C1 , -np.sqrt(pow(A, 2) + pow(B, 2) - pow(C1, 2))) - np.arctan2(A, B) - angle2[0]
        angle3Result[1] = np.arctan2(C2 , -np.sqrt(pow(A, 2) + pow(B, 2) - pow(C2, 2))) - np.arctan2(A, B) - angle2[1]
        angle3Result[2] = np.arctan2(C3 , -np.sqrt(pow(A, 2) + pow(B, 2) - pow(C3, 2))) - np.arctan2(A, B) - angle2[2]
        angle3Result[3] = np.arctan2(C4 , -np.sqrt(pow(A, 2) + pow(B, 2) - pow(C4, 2))) - np.arctan2(A, B) - angle2[3]
        return angle3Result
    
    # Determining rotation matrix from joint 3 to endeffector 
    def _rot_3toE(self, endeffector, angle1, angle2, angle3):
        rotE = np.array([[endeffector[0, 0], endeffector[0, 1], endeffector[0, 2]],
                        [endeffector[1, 0], endeffector[1, 1], endeffector[1, 2]],
                        [endeffector[2, 0], endeffector[2, 1], endeffector[2, 2]]])
        rot1 = np.array([[np.cos(angle1), -np.sin(angle2), 0],
                        [np.sin(angle1), np.cos(angle1), 0],
                        [0, 0, 1]])
        rot2 = np.array([[np.cos(angle2), 0, np.sin(angle2)],
                        [0, 1, 0],
                        [-np.sin(angle2), 0, np.cos(angle2)]])
        rot3 = np.array([[np.cos(angle3), 0, np.sin(angle3)],
                        [0, 1, 0],
                        [-np.sin(angle3), 0, np.cos(angle3)]])
        rot7 = np.array([[np.cos(np.pi / 2), 0, np.sin(np.pi / 2)],
                        [0, 1, 0],
                        [-np.sin(np.pi / 2), 0, np.cos(np.pi / 2)]])
        
        invrot1 = np.linalg.inv(rot1)
        invrot2 = np.linalg.inv(rot2)
        invrot3 = np.linalg.inv(rot3)
        invrot7 = np.linalg.inv(rot7)
        
        rot3toE = np.dot(np.dot(np.dot(np.dot(invrot3, invrot2), invrot1), rotE), invrot7)
        return rot3toE
        
    # Calculating angle 4
    def _angle4(self, rot3toE, angle5):
        angle4Result = np.zeros(2)
        angle4Result[0] = np.arctan2(rot3toE[1, 0] / np.sin(angle5[0]), -rot3toE[2, 0] / np.sin(angle5[0]))
        angle4Result[1] = np.arctan2(rot3toE[1, 0] / np.sin(angle5[1]), -rot3toE[2, 0] / np.sin(angle5[1]))
        return angle4Result
    
    # Calculating angle 5
    def _angle5(self, rot3toE):
        angle5Result = np.zeros(2)
        angle5Result[0] = np.arctan2(np.sqrt(pow(rot3toE[0, 1], 2) + pow(rot3toE[0, 2], 2)), rot3toE[0, 0])
        angle5Result[1] = np.arctan2(-np.sqrt(pow(rot3toE[0, 1], 2) + pow(rot3toE[0, 2], 2)), rot3toE[0, 0])
        return angle5Result

    # Calculating angle 6
    def _angle6(self, rot3toE, angle5):
        angle6Result = np.zeros(2)
        angle6Result[0] = np.arctan2(rot3toE[0, 1] / np.sin(angle5[0]), rot3toE[0, 2] / np.sin(angle5[0]))
        angle6Result[1] = np.arctan2(rot3toE[0, 1] / np.sin(angle5[1]), rot3toE[0, 2] / np.sin(angle5[1]))
        return angle6Result
    
    # Inverse kinematics using position and Euler angles of endeffector
    def inverseKinematics(self, pos, euler):
        self._convertToEndEffector(pos, euler)
        self._wristEffector(self.endeffector, self.lengths)
        angle1list = self._angle1(self.wristeffector)
        self.angles[0] = angle1list[0]
        angle2list = self._angle2(self.wristeffector, angle1list, self.lengths)
        self.angles[1] = angle2list[0]
        angle3list = self._angle3(self.wristeffector, angle1list, angle2list, self.lengths)
        self.angles[2] = angle3list[0]
        rot3toE = self._rot_3toE(self.endeffector, self.angles[0], self.angles[1], self.angles[2])
        angle5list = self._angle5(rot3toE)
        self.angles[4] = angle5list[0]
        angle4list = self._angle4(rot3toE, angle5list)
        self.angles[3] = angle4list[0]
        angle6list = self._angle6(rot3toE, angle5list)
        self.angles[5] = angle6list[0]

def straightTrajectory(start, end, iter):
    points = []
    direction = end - start
    for i in range(iter):
        point = start + (direction * i / (iter - 1))
        points.append(point)
    return points

def circularTrajectory(midpoint, plane, radius, iter):
    points = []
    for i in range(iter):
        angle = i * 2 * np.pi / (iter - 1)
        if plane == "z":
            point = np.array([midpoint[0] + radius * np.cos(angle), midpoint[1] + radius * np.sin(angle), midpoint[2]])
        elif plane == "x":
            point = np.array([midpoint[0], midpoint[1] + radius * np.cos(angle), midpoint[2] + radius * np.sin(angle)])
        elif plane == "y":
            point = np.array([midpoint[0] + radius * np.cos(angle), midpoint[1], midpoint[2] + radius * np.sin(angle)])
        points.append(point)
    return points
    
    
if __name__ == "__main__":
    arm1lengths = np.array([187, 103, 270, 70, 134, 168, 72])
    arm1PosStart = np.array([100, 200, 200])
    arm1PosEnd = np.array([200, 200, 200])
    arm1euler = np.array([0, 0, 90 * DR])
    arm1 = Arm(arm1lengths)
    midpoint = np.array([100, 100, 100])
    circularPointlist = circularTrajectory(midpoint, "z", 50, 50)
    straightPointlist = straightTrajectory(arm1PosStart, arm1PosEnd, 20)
    for point in circularPointlist:
        arm1.inverseKinematics(point, arm1euler)
        print(arm1.angles)
    

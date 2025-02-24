import numpy as np
import matplotlib.pyplot as plt
import math
np.set_printoptions(precision=5, suppress=None)


class ArmL:
    def __init__(self):
        self.L1 = 30.4
        self.L2 = 28.05
        self.L3 = 4.4
        self.theta = np.array([-90, 90, 0, 0, 0, -90])
        self.d = np.array([0, 0, -self.L1, 0, -self.L2, 0])
        self.a = np.array([0, 0, 0, 0, 0, self.L3])
        self.alpha = np.array([-90, -90, 90, -90, 90, 0])
        self.joint_num = len(self.theta) + 1
        self.theta_c = []
    def DH(self, theta, d, a, alpha):
        theta = np.radians(theta)
        alpha = np.radians(alpha)
        dh_matrix = np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        return dh_matrix
    def fk(self, theta_input):
        tip_0 = np.eye(4)
        self.theta_c = self.theta + theta_input
        for i in range(len(self.theta)):
            tip_0 = np.dot(tip_0, self.DH(self.theta[i] + theta_input[i], self.d[i], self.a[i], self.alpha[i]))

        x = tip_0[0, 3]
        y = tip_0[1, 3]
        z = tip_0[2, 3]

        tip_T = np.array([x, y, z])
        tip_R = tip_0[:3, :3]
        tip = np.column_stack((tip_R, tip_T))

        return tip

    def IK(self, ep, body_length, shoulder_width):
        theta = np.zeros(6)
        X, Y, Z = np.zeros(2), np.zeros(2), np.zeros(2)
        R = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
        ep[0, 0] = ep[0, 0] + shoulder_width
        ep[0, 2] = ep[0, 2] - body_length
        unch_RZYX = np.array([[ep[1, 0]], [ep[1, 1]], [ep[1, 2]]])
        ch_RZYX = np.dot(R, unch_RZYX)
        # print(ch_RZYX)
        rot_m = self.eul2rotm(ch_RZYX[0, 0], ch_RZYX[1, 0], ch_RZYX[2, 0])
        # rot_m = self.eul2rotm(ep[1, 0], ep[1, 1], ep[1, 2])
        for i in range(rot_m.shape[0]):
            for j in range(rot_m.shape[1]):
                # print(rot_m[i, j])
                if math.fabs(rot_m[i, j]) < 1e-5:
                    rot_m[i, j] = 0
        # print("rotation matrix : ")
        # print(rot_m)
        end_vectorX = np.array(rot_m[:, 0])
        end_vectorY = np.array(rot_m[:, 1])
        end_vectorZ = np.array(rot_m[:, 2])
        Mx, My, Mz = end_vectorX
        Nx, Ny, Nz = end_vectorY
        Ox, Oy, Oz = end_vectorZ
        # qx, qy, qz = ep[0, 0], ep[0, 1], ep[0, 2]
        unch_XYZ = np.array([[ep[0, 0]], [ep[0, 1]], [ep[0, 2]]], dtype=np.float32)
        ch_XYZ = np.dot(R, unch_XYZ)
        print(unch_XYZ)
        # print(R)
        print(ch_XYZ)
        qx, qy, qz = ch_XYZ[0, 0], ch_XYZ[1, 0], ch_XYZ[2, 0]
        # print(qx, qy, qz)
        d3, d5, a6 = self.L1, self.L2, self.L3
        ####  calculate wrist point  #####
        K1 = np.linalg.norm([Mx, My, Mz])
        Pw_x = qx - a6 * Mx / K1
        Pw_y = qy - a6 * My / K1
        Pw_z = qz - a6 * Mz / K1
        ####  calculate elbow point  #####
        V1 = np.array([Pw_x, Pw_y, Pw_z])
        V2 = np.array([Ox, Oy, Oz])
        Line_vector = np.cross(V1, V2)
        if np.linalg.norm([Pw_x, Pw_y, Pw_z]) == (d3 + d5):
            for i in range(2):
                X[i] = (Pw_x / (d3 + d5)) * d3
                Y[i] = (Pw_y / (d3 + d5)) * d3
                Z[i] = (Pw_z / (d3 + d5)) * d3
        else:
            if V2[0] != 0:
                YY = -(d3 ** 2 - d5 ** 2 + Pw_x ** 2 + Pw_y ** 2 + Pw_z ** 2 - (
                            2 * Pw_x * (Pw_x * V2[0] + Pw_y * V2[1] + Pw_z * V2[2])) / V2[0]) / (
                                 (2 * Pw_x * V2[1]) / V2[0] - 2 * Pw_y)
                XX = -(V2[1] * (YY - Pw_y) - Pw_z * V2[2] - Pw_x * V2[0]) / V2[0]
                Ta = (Pw_y * V2[0] - Pw_x * V2[1]) ** 2 + (Pw_z * V2[0] - Pw_x * V2[2]) ** 2 + (
                            Pw_z * V2[1] - Pw_y * V2[2]) ** 2
                Tb = (2 * (Pw_z * V2[0] - Pw_x * V2[2]) * (d3 ** 2 - d5 ** 2 + Pw_x ** 2 + Pw_y ** 2 + Pw_z ** 2 - (
                            2 * Pw_x * (Pw_x * V2[0] + Pw_y * V2[1] + Pw_z * V2[2])) / V2[0])) / (
                                 2 * Pw_y - (2 * Pw_x * V2[1]) / V2[0]) - (2 * (Pw_z * V2[1] - Pw_y * V2[2]) * (
                            Pw_x * V2[0] + Pw_z * V2[2] + V2[1] * (Pw_y - (
                                d3 ** 2 - d5 ** 2 + Pw_x ** 2 + Pw_y ** 2 + Pw_z ** 2 - (
                                    2 * Pw_x * (Pw_x * V2[0] + Pw_y * V2[1] + Pw_z * V2[2])) / V2[0]) / (
                                                                               2 * Pw_y - (2 * Pw_x * V2[1]) / V2[0])))) / \
                     V2[0]
                Tc = (Pw_x * V2[0] + Pw_z * V2[2] + V2[1] * (Pw_y - (
                            d3 ** 2 - d5 ** 2 + Pw_x ** 2 + Pw_y ** 2 + Pw_z ** 2 - (
                                2 * Pw_x * (Pw_x * V2[0] + Pw_y * V2[1] + Pw_z * V2[2])) / V2[0]) / (
                                                                         2 * Pw_y - (2 * Pw_x * V2[1]) / V2[0]))) ** 2 / V2[
                         0] ** 2 - d3 ** 2 + (d3 ** 2 - d5 ** 2 + Pw_x ** 2 + Pw_y ** 2 + Pw_z ** 2 - (
                            2 * Pw_x * (Pw_x * V2[0] + Pw_y * V2[1] + Pw_z * V2[2])) / V2[0]) ** 2 / (
                                 2 * Pw_y - (2 * Pw_x * V2[1]) / V2[0]) ** 2
                # print(YY, XX, Ta, Tb, Tc)
                if (Tb ** 2 - 4 * Ta * Tc) < 0:
                    T1 = (-Tb + 0) / (2 * Ta)
                    T2 = (-Tb - 0) / (2 * Ta)
                else:
                    T1 = (-Tb + np.sqrt(Tb ** 2 - 4 * Ta * Tc)) / (2 * Ta)
                    T2 = (-Tb - np.sqrt(Tb ** 2 - 4 * Ta * Tc)) / (2 * Ta)
                # print(T1, T2)
                X[0], Y[0], Z[0] = XX + Line_vector[0] * T1, YY + Line_vector[1] * T1, Line_vector[2] * T1
                X[1], Y[1], Z[1] = XX + Line_vector[0] * T2, YY + Line_vector[1] * T2, Line_vector[2] * T2
            elif V2[2] != 0:
                XX = -(d3 ** 2 - d5 ** 2 + Pw_x ** 2 + Pw_y ** 2 + Pw_z ** 2 - (
                            2 * Pw_z * (Pw_x * V2[0] + Pw_y * V2[1] + Pw_z * V2[2])) / V2[2]) / (
                                 (2 * Pw_z * V2[0]) / V2[2] - 2 * Pw_x)
                ZZ = -(V2[0] * (XX - Pw_x) - Pw_z * V2[2] - Pw_y * V2[1]) / (V2[2])
                Ta = (Pw_y * V2[0] - Pw_x * V2[1]) ** 2 + (Pw_z * V2[0] - Pw_x * V2[2]) ** 2 + (
                            Pw_z * V2[1] - Pw_y * V2[2]) ** 2
                Tb = - (2 * (Pw_z * V2[1] - Pw_y * V2[2]) * (
                            V2[2] * d5 ** 2 - V2[2] * d3 ** 2 - V2[2] * Pw_x ** 2 + 2 * V2[0] * Pw_x * Pw_z - V2[
                        2] * Pw_y ** 2 + 2 * V2[1] * Pw_y * Pw_z + V2[2] * Pw_z ** 2)) / (
                                 2 * Pw_z * V2[0] - 2 * Pw_x * V2[2]) - (2 * (Pw_y * V2[0] - Pw_x * V2[1]) * (
                            Pw_x * V2[0] - XX * V2[0] + Pw_y * V2[1] + Pw_z * V2[2])) / V2[2]
                Tc = (V2[2] * d5 ** 2 - V2[2] * d3 ** 2 - V2[2] * Pw_x ** 2 + 2 * V2[0] * Pw_x * Pw_z - V2[
                    2] * Pw_y ** 2 + 2 * V2[1] * Pw_y * Pw_z + V2[2] * Pw_z ** 2) ** 2 / (
                                 4 * (Pw_z * V2[0] - Pw_x * V2[2]) ** 2) - d3 ** 2 + (
                                 Pw_x * V2[0] - XX * V2[0] + Pw_y * V2[1] + Pw_z * V2[2]) ** 2 / V2[2] ** 2
                if (Tb ** 2 - 4 * Ta * Tc) < 0:
                    T1 = (-Tb + 0) / (2 * Ta)
                    T2 = (-Tb - 0) / (2 * Ta)
                else:
                    T1 = (-Tb + np.sqrt(Tb ** 2 - 4 * Ta * Tc)) / (2 * Ta)
                    T2 = (-Tb - np.sqrt(Tb ** 2 - 4 * Ta * Tc)) / (2 * Ta)
                X[0], Y[0], Z[0] = XX + Line_vector[0] * T1, Line_vector[1] * T1, ZZ + Line_vector[2] * T1
                X[1], Y[1], Z[1] = XX + Line_vector[0] * T2, Line_vector[1] * T2, ZZ + Line_vector[2] * T2
            elif V2[1] != 0:
                ZZ = -(d3 ** 2 - d5 ** 2 + Pw_x ** 2 + Pw_y ** 2 + Pw_z ** 2 - (
                        2 * Pw_y * (Pw_x * V2[0] + Pw_y * V2[1] + Pw_z * V2[2])) / V2[1]) / (
                             (2 * Pw_y * V2[2]) / V2[1] - 2 * Pw_z)
                YY = -(V2[2] * (ZZ - Pw_z) - Pw_y * V2[1] - Pw_x * V2[0]) / (V2[1])
                Ta = (Pw_y * V2[0] - Pw_x * V2[1]) ** 2 + (Pw_z * V2[0] - Pw_x * V2[2]) ** 2 + (
                        Pw_z * V2[1] - Pw_y * V2[2]) ** 2
                Tb = (2 * (Pw_y * V2[0] - Pw_x * V2[1]) * (
                        V2[1] * d5 ** 2 - V2[1] * d3 ** 2 - V2[1] * Pw_x ** 2 + 2 * V2[0] * Pw_x * Pw_y + V2[
                    1] * Pw_y ** 2 + 2 * V2[2] * Pw_y * Pw_z - V2[1] * Pw_z ** 2)) / (
                             2 * Pw_z * V2[1] - 2 * Pw_y * V2[2]) + (
                             2 * (Pw_z * V2[0] - Pw_x * V2[2]) * (
                             Pw_x * V2[0] - ZZ * V2[2] + Pw_y * V2[1] + Pw_z * V2[2])) / V2[1]
                Tc = (
                             V2[1] * d5 ** 2 - V2[1] * d3 ** 2 - V2[1] * Pw_x ** 2 + 2 * V2[0] * Pw_x * Pw_y + V2[
                         1] * Pw_y ** 2 + 2 * V2[2] * Pw_y * Pw_z - V2[1] * Pw_z ** 2) ** 2 / (
                             4 * (Pw_z * V2[1] - Pw_y * V2[2]) ** 2) - d3 ** 2 + (
                             Pw_x * V2[0] - ZZ * V2[2] + Pw_y * V2[1] + Pw_z * V2[2]) ** 2 / V2[1] ** 2
                if (Tb ** 2 - 4 * Ta * Tc) < 0:
                    T1 = (-Tb + 0) / (2 * Ta)
                    T2 = (-Tb - 0) / (2 * Ta)
                else:
                    T1 = (-Tb + np.sqrt(Tb ** 2 - 4 * Ta * Tc)) / (2 * Ta)
                    T2 = (-Tb - np.sqrt(Tb ** 2 - 4 * Ta * Tc)) / (2 * Ta)
                X[0], Y[0], Z[0] = Line_vector[0] * T1, YY + Line_vector[1] * T1, ZZ + Line_vector[2] * T1
                X[1], Y[1], Z[1] = Line_vector[0] * T2, YY + Line_vector[1] * T2, ZZ + Line_vector[2] * T2
        for i in range(2):
            if math.fabs(X[i]) < 1e-10:
                X[i] = 0
            if math.fabs(Y[i]) < 1e-10:
                Y[i] = 0
            if math.fabs(Z[i]) < 1e-10:
                Z[i] = 0
        #### 選解 ####
        if Z[0] < Z[1]:
            Pe_x = X[0]
            Pe_y = Y[0]
            Pe_z = Z[0]
            # print("1")
        elif Z[0] == Z[1]:
            if Y[0] >= Y[1]:
                Pe_x = X[1]
                Pe_y = Y[1]
                Pe_z = Z[1]
                # print("2")
            else:
                Pe_x = X[0]
                Pe_y = Y[0]
                Pe_z = Z[0]
                # print("3")
        else:
            Pe_x = X[1]
            Pe_y = Y[1]
            Pe_z = Z[1]
        # print(Pe_x, Pe_y, Pe_z)
        #############  解6軸   ###########
        ## theta 4 ##
        Dw = math.sqrt(Pw_x ** 2 + Pw_y ** 2 + Pw_z ** 2)
        if Dw > (d3 ** 2 + d5 ** 2):
            Dw = d3 ** 2 + d5 ** 2

        CT4 = (d3 ** 2 + d5 ** 2 - Dw ** 2) / (2 * d3 * d5)
        if math.fabs(math.fabs(CT4) - 1) < 0.0001:
            theta[3] = 0
        else:
            theta[3] = (math.pi - math.acos(CT4)) * -1

        # theta1
        theta[0] = math.atan2(Pe_x, -Pe_y)

        # theta2
        if Pe_x == 0:
            theta[1] = math.atan2(-Pe_z, -Pe_y)
        else:
            theta[1] = math.atan2(-Pe_z, Pe_x / math.sin(theta[0]))
        # print(theta[3], theta[0], theta[1])
        # theta3
        if theta[1] == (np.pi / 2) or theta[3] == 0:
            if qx > 0:
                theta[2] = -math.pi / 2
            elif qx < 0:
                theta[2] = math.pi / 2
            else:
                theta[2] = 0
        else:
            CT3 = (Pw_z - (d3 * math.cos(theta[1] + math.pi / 2) + d5 * math.cos(theta[1] + math.pi / 2) * math.cos(
                theta[3]))) / (-d5 * math.sin(theta[1] + math.pi / 2) * math.sin(theta[3]))
            # print(CT3)
            if math.fabs(math.fabs(CT3) - 1) < 0.0001:
                theta[2] = 0
            else:
                theta[2] = math.acos(CT3)

        # Check Pw_y
        check_J3 = d5 * math.cos(theta[0]) * math.cos(theta[2]) * math.sin(theta[1]) * math.sin(theta[3]) - d5 * math.cos(
            theta[0]) * math.cos(theta[1]) * math.cos(theta[3]) - d5 * math.sin(theta[0]) * math.sin(theta[2]) * math.sin(
            theta[3]) - d3 * math.cos(theta[0]) * math.cos(theta[1])
        if math.fabs(Pw_y - check_J3) > 1e-1:
            theta[2] *= -1

        # theta6
        De2e = math.sqrt((Pe_x - qx) ** 2 + (Pe_y - qy) ** 2 + (Pe_z - qz) ** 2)
        if De2e > (a6 ** 2 + d5 ** 2):
            De2e = a6 ** 2 + d5 ** 2
        # print(De2e)
        CT6 = (d5 ** 2 + a6 ** 2 - De2e ** 2) / (2 * d5 * a6)
        # print(CT6)
        if math.fabs(math.fabs(CT6) - 1) < 0.0001:
            print('aaa')
            theta[5] = 0
        else:
            theta[5] = math.pi - math.acos(CT6)

        # theta5
        Ax = Nz * math.cos(theta[5]) * math.sin(theta[1]) * math.sin(theta[3]) + Mz * math.sin(theta[1]) * math.sin(
            theta[3]) * math.sin(theta[5]) - Nz * math.cos(theta[1]) * math.cos(theta[2]) * math.cos(theta[3]) * math.cos(
            theta[5]) - Mz * math.cos(theta[1]) * math.cos(theta[2]) * math.cos(theta[3]) * math.sin(
            theta[5]) - Nx * math.cos(theta[0]) * math.cos(theta[3]) * math.cos(theta[5]) * math.sin(
            theta[2]) + Ny * math.cos(theta[0]) * math.cos(theta[1]) * math.cos(theta[5]) * math.sin(
            theta[3]) - Mx * math.cos(theta[0]) * math.cos(theta[3]) * math.sin(theta[2]) * math.sin(
            theta[5]) + My * math.cos(theta[0]) * math.cos(theta[1]) * math.sin(theta[3]) * math.sin(
            theta[5]) - Nx * math.cos(theta[1]) * math.cos(theta[5]) * math.sin(theta[0]) * math.sin(
            theta[3]) - Ny * math.cos(theta[3]) * math.cos(theta[5]) * math.sin(theta[0]) * math.sin(
            theta[2]) - Mx * math.cos(theta[1]) * math.sin(theta[0]) * math.sin(theta[3]) * math.sin(
            theta[5]) - My * math.cos(theta[3]) * math.sin(theta[0]) * math.sin(theta[2]) * math.sin(
            theta[5]) + My * math.cos(theta[0]) * math.cos(theta[2]) * math.cos(theta[3]) * math.sin(theta[1]) * math.sin(
            theta[5]) - Nx * math.cos(theta[2]) * math.cos(theta[3]) * math.cos(theta[5]) * math.sin(theta[0]) * math.sin(
            theta[1]) - Mx * math.cos(theta[2]) * math.cos(theta[3]) * math.sin(theta[0]) * math.sin(theta[1]) * math.sin(
            theta[5]) + Ny * math.cos(theta[0]) * math.cos(theta[2]) * math.cos(theta[3]) * math.cos(theta[5]) * math.sin(
            theta[1])
        Ay = Nz * math.cos(theta[1]) * math.cos(theta[5]) * math.sin(theta[2]) - Mx * math.cos(theta[0]) * math.cos(
            theta[2]) * math.sin(theta[5]) - Ny * math.cos(theta[2]) * math.cos(theta[5]) * math.sin(
            theta[0]) - Nx * math.cos(theta[0]) * math.cos(theta[2]) * math.cos(theta[5]) - My * math.cos(
            theta[2]) * math.sin(theta[0]) * math.sin(theta[5]) + Mz * math.cos(theta[1]) * math.sin(theta[2]) * math.sin(
            theta[5]) - Ny * math.cos(theta[0]) * math.cos(theta[5]) * math.sin(theta[1]) * math.sin(
            theta[2]) - My * math.cos(theta[0]) * math.sin(theta[1]) * math.sin(theta[2]) * math.sin(
            theta[5]) + Nx * math.cos(theta[5]) * math.sin(theta[0]) * math.sin(theta[1]) * math.sin(
            theta[2]) + Mx * math.sin(theta[0]) * math.sin(theta[1]) * math.sin(theta[2]) * math.sin(theta[5])

        theta[4] = math.atan2(Ay, Ax)

        # Check Oz 和 qz
        check_zz = math.sin(theta[1]) * math.sin(theta[3]) * math.sin(theta[4]) - math.cos(theta[1]) * math.cos(
            theta[4]) * math.sin(theta[2]) - math.cos(theta[1]) * math.cos(theta[2]) * math.cos(theta[3]) * math.sin(
            theta[4])
        check_yy = math.sin(theta[5]) * (
                math.sin(theta[0]) * math.sin(theta[2]) * math.sin(theta[3]) + math.cos(theta[0]) * math.cos(
            theta[1]) * math.cos(theta[3]) - math.cos(theta[0]) * math.cos(theta[2]) * math.sin(theta[1]) * math.sin(
            theta[3])) - math.cos(theta[5]) * (
                           math.cos(theta[2]) * math.sin(theta[0]) * math.sin(theta[4]) - math.cos(theta[0]) * math.cos(
                       theta[1]) * math.cos(theta[4]) * math.sin(theta[3]) + math.cos(theta[3]) * math.cos(
                       theta[4]) * math.sin(theta[0]) * math.sin(theta[2]) + math.cos(theta[0]) * math.sin(
                       theta[1]) * math.sin(theta[2]) * math.sin(theta[4]) - math.cos(theta[0]) * math.cos(
                       theta[2]) * math.cos(theta[3]) * math.cos(theta[4]) * math.sin(theta[1]))
        check_J6 = a6 * math.cos(theta[1]) * math.sin(theta[2]) * math.sin(theta[4]) * math.sin(theta[5]) - d5 * math.cos(
            theta[3]) * math.sin(theta[1]) - a6 * math.cos(theta[3]) * math.cos(theta[5]) * math.sin(
            theta[1]) - d5 * math.cos(theta[1]) * math.cos(theta[2]) * math.sin(theta[3]) - a6 * math.cos(
            theta[1]) * math.cos(theta[2]) * math.cos(theta[5]) * math.sin(theta[3]) - d3 * math.sin(
            theta[1]) + a6 * math.cos(theta[4]) * math.sin(theta[1]) * math.sin(theta[3]) * math.sin(
            theta[5]) - a6 * math.cos(theta[1]) * math.cos(theta[2]) * math.cos(theta[3]) * math.cos(theta[4]) * math.sin(
            theta[5])

        if (math.fabs(qz - check_J6) > 1e-3) or (math.fabs(check_zz - Oz) > 1e-3) or (math.fabs(check_yy - Ny) > 1e-3):
            theta[5] *= -1
            Ax = Nz * math.cos(theta[5]) * math.sin(theta[1]) * math.sin(theta[3]) + Mz * math.sin(theta[1]) * math.sin(
                theta[3]) * math.sin(theta[5]) - Nz * math.cos(theta[1]) * math.cos(theta[2]) * math.cos(
                theta[3]) * math.cos(theta[5]) - Mz * math.cos(theta[1]) * math.cos(theta[2]) * math.cos(
                theta[3]) * math.sin(theta[5]) - Nx * math.cos(theta[0]) * math.cos(theta[3]) * math.cos(
                theta[5]) * math.sin(theta[2]) + Ny * math.cos(theta[0]) * math.cos(theta[1]) * math.cos(
                theta[5]) * math.sin(theta[3]) - Mx * math.cos(theta[0]) * math.cos(theta[3]) * math.sin(
                theta[2]) * math.sin(theta[5]) + My * math.cos(theta[0]) * math.cos(theta[1]) * math.sin(
                theta[3]) * math.sin(theta[5]) - Nx * math.cos(theta[1]) * math.cos(theta[5]) * math.sin(
                theta[0]) * math.sin(theta[3]) - Ny * math.cos(theta[3]) * math.cos(theta[5]) * math.sin(
                theta[0]) * math.sin(theta[2]) - Mx * math.cos(theta[1]) * math.sin(theta[0]) * math.sin(
                theta[3]) * math.sin(theta[5]) - My * math.cos(theta[3]) * math.sin(theta[0]) * math.sin(
                theta[2]) * math.sin(theta[5]) + My * math.cos(theta[0]) * math.cos(theta[2]) * math.cos(
                theta[3]) * math.sin(theta[1]) * math.sin(theta[5]) - Nx * math.cos(theta[2]) * math.cos(
                theta[3]) * math.cos(theta[5]) * math.sin(theta[0]) * math.sin(theta[1]) - Mx * math.cos(
                theta[2]) * math.cos(theta[3]) * math.sin(theta[0]) * math.sin(theta[1]) * math.sin(
                theta[5]) + Ny * math.cos(theta[0]) * math.cos(theta[2]) * math.cos(theta[3]) * math.cos(
                theta[5]) * math.sin(theta[1])
            Ay = Nz * math.cos(theta[1]) * math.cos(theta[5]) * math.sin(theta[2]) - Mx * math.cos(theta[0]) * math.cos(
                theta[2]) * math.sin(theta[5]) - Ny * math.cos(theta[2]) * math.cos(theta[5]) * math.sin(
                theta[0]) - Nx * math.cos(theta[0]) * math.cos(theta[2]) * math.cos(theta[5]) - My * math.cos(
                theta[2]) * math.sin(theta[0]) * math.sin(theta[5]) + Mz * math.cos(theta[1]) * math.sin(
                theta[2]) * math.sin(theta[5]) - Ny * math.cos(theta[0]) * math.cos(theta[5]) * math.sin(
                theta[1]) * math.sin(theta[2]) - My * math.cos(theta[0]) * math.sin(theta[1]) * math.sin(
                theta[2]) * math.sin(theta[5]) + Nx * math.cos(theta[5]) * math.sin(theta[0]) * math.sin(
                theta[1]) * math.sin(theta[2]) + Mx * math.sin(theta[0]) * math.sin(theta[1]) * math.sin(
                theta[2]) * math.sin(theta[5])

            theta[4] = math.atan2(Ay, Ax)
        for i in range(6):
            # 將接近 2 * pi 的值設為 0
            if math.fabs(theta[i] - 2 * math.pi) < 0.00001:
                theta[i] = 0
            # 如果超過 2 * pi，則減去 2 * pi，使其範圍保持在 0 到 2 * pi
            if theta[i] > 2 * math.pi:
                theta[i] = theta[i] - 2 * math.pi

        # 將弧度值轉換為角度
        for i in range(6):
            theta[i] = theta[i] * 180 / math.pi
        return theta

    def rotm2eul(self, rotation_matrix, degrees=True):
        sy = np.sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = 0

        euler_angles = np.array([x, y, z])

        if degrees:
            euler_angles = np.degrees(euler_angles)

        return euler_angles
    def eul2rotm(self, a, b, r, degree=True):
        if degree:
            a = np.radians(a)
            b = np.radians(b)
            r = np.radians(r)
        # print(a, "  ", b, "  ", r)
        rotation_matrix = np.array([
            [np.cos(a) * np.cos(b), np.cos(a) * np.sin(b) * np.sin(r) - np.sin(a) * np.cos(r),
             np.cos(a) * np.sin(b) * np.cos(r) + np.sin(a) * np.sin(r)],
            [np.sin(a) * np.cos(b), np.sin(a) * np.sin(b) * np.sin(r) + np.cos(a) * np.cos(r),
             np.sin(a) * np.sin(b) * np.cos(r) - np.cos(a) * np.sin(r)],
            [-np.sin(b), np.cos(b) * np.sin(r), np.cos(b) * np.cos(r)]
        ])
        return rotation_matrix
    def draw_coordinate(self, ax, name, p, R=np.eye(3), scale=25):
        ax.quiver(p[0], p[1], p[2], R[0, 0], R[1, 0], R[2, 0], color='r', linewidth=2, length=scale)
        ax.quiver(p[0], p[1], p[2], R[0, 1], R[1, 1], R[2, 1], color='g', linewidth=2, length=scale)
        ax.quiver(p[0], p[1], p[2], R[0, 2], R[1, 2], R[2, 2], color='b', linewidth=2, length=scale)
        ax.text(p[0] + R[0, 0] * scale, p[1] + R[1, 0] * scale, p[2] + R[2, 0] * scale, 'x_' + name)
        ax.text(p[0] + R[0, 1] * scale, p[1] + R[1, 1] * scale, p[2] + R[2, 1] * scale, 'y_' + name)
        ax.text(p[0] + R[0, 2] * scale, p[1] + R[1, 2] * scale, p[2] + R[2, 2] * scale, 'z_' + name)

    def draw_line(self, ax, p, q):
        ax.plot([p[0], q[0]], [p[1], q[1]], [p[2], q[2]], 'k-', linewidth=2)

    def draw_marker(self, ax, p, label):
        ax.plot([p[0]], [p[1]], [p[2]], 'r.', markersize=15)
        ax.text(p[0]+5, p[1]+5, p[2]+5, label, fontsize=9)

    def plot_sim(self, theta_input):
        tip = self.fk(theta_input)
        tip_rot = tip[:, :3]
        tip_pos = tip[:, 3]
        eulZYX = self.rotm2eul(tip_rot, True)
        posCBA = np.array([eulZYX[2], eulZYX[1], eulZYX[0]])

        print("tip:")
        print(tip)
        print("tip_rot:")
        print(tip_rot)
        print("tip_pos:", tip_pos)
        print("eulZYX:", eulZYX)
        print("posCBA:", posCBA)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        T = [np.eye(4)] * self.joint_num
        P = [np.zeros(3)] * self.joint_num
        R_matrices = [np.eye(3)] * self.joint_num

        T[0] = np.eye(4)
        P[0] = T[0][:3, 3]
        R_matrices[0] = T[0][:3, :3]

        for i in range(1, self.joint_num):
            T[i] = np.dot(T[i - 1], self.DH(self.theta_c[i - 1], self.d[i - 1], self.a[i - 1], self.alpha[i - 1]))
            P[i] = T[i][:3, 3]
            R_matrices[i] = T[i][:3, :3]

        for i in range(self.joint_num):
            P[i] = np.array([P[i][2], P[i][0], P[i][1]])  # Swap the coordinates

        self.draw_coordinate(ax, '0', P[0], R_matrices[0])
        for k in range(1, self.joint_num):
            self.draw_line(ax, P[k - 1], P[k])
            if k == 4:
                self.draw_marker(ax, P[k - 1], 'Elbow')
            elif k == 6:
                self.draw_marker(ax, P[k - 1], 'Wrist')
            elif k == 1:
                self.draw_marker(ax, P[k - 1], 'Shoulder')
            if k == 6:
                self.draw_coordinate(ax, str(k), P[k], R_matrices[k])

        ax.set_xlim(xmin=-70, xmax=70)
        ax.set_ylim(ymin=-70, ymax=70)
        ax.set_zlim(zmin=-70, zmax=70)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        # ax.view_init(elev=116, azim=-90)
        plt.show()
    def sim_to_body(self, body_length, shoulder_width):
        base = [0, 0, 0]
        shoulder_center = [0, 0, body_length]
        right_shoulder = [shoulder_width, 0, body_length]
        left_shoulder = [-shoulder_width, 0, body_length]
        # draw picture
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot([base[0], shoulder_center[0]], [base[1], shoulder_center[1]], [base[2], shoulder_center[2]], 'k-')
        ax.plot([shoulder_center[0], right_shoulder[0]], [shoulder_center[1], right_shoulder[1]], [shoulder_center[2], right_shoulder[2]], 'k-')
        ax.plot([shoulder_center[0], left_shoulder[0]], [shoulder_center[1], left_shoulder[1]], [shoulder_center[2], left_shoulder[2]], 'k-')

        ax.scatter(*base, color='blue', s=50, label='Base')
        ax.scatter(*shoulder_center, color='orange', s=50, label='Shoulder Center')
        ax.scatter(*right_shoulder, color='green', s=50, label='Right Shoulder')
        ax.scatter(*left_shoulder, color='red', s=50, label='Left Shoulder')

        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')
        ax.legend()

        plt.show()



def main():
    LA = ArmL()
    # theta_input = np.array([62.38748499,   50.99774714, -118.02067641,   71.90069844, 0, 0])
    # ep = np.array([[-19.4746, 40.5699, -5.1251], [-115.4633, -21.0240, 16.1558]])
    # ep = np.array([[26.77318, -30.65429, -12.9771], [0, 0, 0]])
    ep = np.array([[-3, 38, 10], [0, -160, -90]], dtype=np.float32)
    body_length = 40.5
    shoulder_width = 21.3
    angles = LA.IK(ep, body_length, shoulder_width)
    # print(LA.fk(theta_input))
    print(angles)
    LA.plot_sim(angles)
    # LA.sim_to_body(body_length, shoulder_width)

if __name__ == '__main__':
    main()
import math

tan30 = 1.0 / math.sqrt(3.0)
tan30_05 = tan30 * 0.5
tan60 = math.sqrt(3.0)
sin30 = 0.5
cos30 = math.sqrt(3.0) / 2.0
cos120 = -0.5
sin120 = math.sqrt(3.0) / 2.0

class Kinematic():
    def __init__(self, rd_rf = 0.0, rd_re = 0.0, rd_e = 0.0, rd_f = 0.0, rd_of = 0.0):
        #robot parameter
        self.__rd_rf = rd_rf
        self.__rd_re = rd_re
        self.__rd_e = rd_e
        self.__rd_f = rd_f
        self.__rd_of = rd_of
        #robot state
        self.__theta1 = 0.0
        self.__theta2 = 0.0
        self.__theta3 = 0.0
        self.__x = 0.0
        self.__y = 0.0
        self.__z = 0.0
        #robot component state
        self.ball_top1 = 0.0
        self.ball_top2 = 0.0
        self.ball_top3 = 0.0
        self.re12 = 0.0
        self.re34 = 0.0
        self.re56 = 0.0
        self.re_ball = 0.0
        self.ball_moving = 0.0
        #robot component defaunt state
        self.__defauld_ball_top = 0.0
        self.__defauld_ball_moving = 0.0

        if rd_rf != 0.0 and rd_re != 0.0 and rd_e != 0.0 and rd_f != 0.0:
            self.forward(0.0, 0.0, 0.0)
            self.__defauld_ball_top = self.ball_top1
            self.__defauld_ball_moving = self.ball_moving

    def set_robot_parameter(self, rd_rf = 0.0, rd_re = 0.0, rd_e = 0.0, rd_f = 0.0, rd_of = 0.0):
        self.__rd_rf = rd_rf
        self.__rd_re = rd_re
        self.__rd_e = rd_e
        self.__rd_f = rd_f
        self.__rd_of = rd_of

        self.forward(0.0, 0.0, 0.0)
        self.__defauld_ball_top = self.ball_top1
        self.__defauld_ball_moving = self.ball_moving


    def get_theta(self):
        return self.__theta1, self.__theta2, self.__theta3

    def get_point(self):
        return self.__x, self.__y, self.__z - self.__rd_of

    def get_component_state(self):
        return [self.ball_top1 - self.__defauld_ball_top, self.ball_top2 - self.__defauld_ball_top, self.ball_top3 - self.__defauld_ball_top,
            self.re12, self.re34, self.re56, self.re_ball, self.ball_moving - self.__defauld_ball_moving]
    
    def forward(self, theta1, theta2, theta3):
        self.__theta1 = math.radians(theta1)
        self.__theta2 = math.radians(theta2)
        self.__theta3 = math.radians(theta3)

        t = (self.__rd_f - self.__rd_e) * tan30 / 2.0

        y1 = -(t + self.__rd_rf * math.cos(self.__theta1))
        z1 = -self.__rd_rf * math.sin(self.__theta1)

        y2 = (t + self.__rd_rf * math.cos(self.__theta2)) * sin30
        x2 = y2 * tan60
        z2 = -self.__rd_rf * math.sin(self.__theta2)

        y3 = (t + self.__rd_rf * math.cos(self.__theta3)) * sin30
        x3 = -y3 * tan60
        z3 = -self.__rd_rf * math.sin(self.__theta3)

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = y1 * y1 + z1 * z1
        w2 = x2 * x2 + y2 * y2 + z2 * z2
        w3 = x3 * x3 + y3 * y3 + z3 * z3

        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        a = a1 * a1 + a2 * a2 + dnm * dnm
        b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
        c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - self.__rd_re * self.__rd_re)

        d = b * b - 4.0 * a * c
        if d < 0:
            return False

        self.__z = -0.5 * (b + math.sqrt(d)) / a
        self.__x = (a1 * self.__z + b1) / dnm
        self.__y = (a2 * self.__z + b2) / dnm

        self.__calculate_component_state()

        return True

    def __inverseTheta(self, x0, y0, z0):
        y1 = -0.5 * tan30 * self.__rd_f
        y0 -= 0.5 * tan30 * self.__rd_e

        a = (x0 * x0 + y0 * y0 + z0 * z0 + self.__rd_rf * self.__rd_rf - self.__rd_re * self.__rd_re - y1 * y1) / (2.0 * z0)
        b = (y1 - y0) / z0

        d = -(a + b * y1) * (a + b * y1) + b * b * self.__rd_rf * self.__rd_rf + self.__rd_rf * self.__rd_rf

        if d < 0:
            return False

        yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1.0)
        zj = a + b * yj

        theta = math.atan(-zj / (y1 - yj))
        if yj > y1:
            theta += math.pi
        else:
            theta += 0.0

        return theta

    def inverse(self, x, y, z):
        self.__x = x
        self.__y = y
        self.__z = z + self.__rd_of
        t1 = self.__inverseTheta(self.__x, self.__y, self.__z)
        t2 = self.__inverseTheta(self.__x * cos120 + self.__y * sin120, self.__y * cos120 - self.__x * sin120, self.__z)
        t3 = self.__inverseTheta(self.__x * cos120 - self.__y * sin120, self.__y * cos120 + self.__x * sin120, self.__z)

        if t1 != False:
            self.__theta1 = t1
        else:
            return False
        if t2 != False:
            self.__theta2 = t2
        else:
            return False
        if t3 != False:
            self.__theta3 = t3
        else:
            return False
        
        self.__calculate_component_state()

        return True

    def __calculate_component_state_theta(self, x0, y0, z0, theta):
        l1 = 0.5 * tan30 * self.__rd_f
        l2 = 0.5 * tan30 * self.__rd_e

        f1_y = -l1
        f1_z = 0

        e11_y = y0 - l2
        e11_z = z0

        re = math.asin(x0 / self.__rd_re)

        l_je = math.sqrt(self.__rd_re * self.__rd_re - x0 * x0)
        l_fe = math.sqrt((e11_z - f1_z) * (e11_z - f1_z) + (e11_y - f1_y) * (e11_y - f1_y))

        ball_top = math.acos((self.__rd_rf * self.__rd_rf + l_je * l_je - l_fe * l_fe) / (2 * self.__rd_rf * l_je))

        return ball_top, re

    def __calculate_component_state(self):
        self.ball_top1, self.re12 = self.__calculate_component_state_theta(self.__x, self.__y, self.__z, self.__theta1)
        self.ball_top2, self.re34 = self.__calculate_component_state_theta(self.__x * cos120 + self.__y * sin120, self.__y * cos120 - self.__x * sin120, self.__z, self.__theta2)
        self.ball_top3, self.re56 = self.__calculate_component_state_theta(self.__x * cos120 - self.__y * sin120, self.__y * cos120 + self.__x * sin120, self.__z, self.__theta3)
        self.re_ball = -self.re12
        if self.__theta1 < 0:
            self.ball_moving = math.pi - abs(self.__theta1) - self.ball_top1
        else:
            self.ball_moving = math.pi + abs(self.__theta1) - self.ball_top1


class Interpolator():
    def __init__(self) -> None:
        pass
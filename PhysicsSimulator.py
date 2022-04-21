import math
import copy

def toVector(scalar, direction):
    if scalar != 0:
        k = direction.length() / scalar
        F = emptyVector(len(direction))
        for j, i in enumerate(direction):
            F[j] = i / k
        return Vector(F)
    else:
        return emptyVector(len(direction))


def emptyVector(directions_len):
    a = []
    for _ in range(directions_len):
        a.append(0)
    return Vector(a)


class Location(list):
    def __add__(self, other):
        out_list = map(lambda x, y: x + y, self, other)
        return Location(out_list)

    def __sub__(self, other):
        out_list = map(lambda x, y: x - y, self, other)
        return Location(out_list)

    def __neg__(self):
        out_list = map(lambda x: -x, self)
        return Location(out_list)

    def __truediv__(self, other):
        out_list = map(lambda x: x / other, self)
        return Location(out_list)

    def __mul__(self, other):
        out_list = map(lambda x: x * other, self)
        return Location(out_list)


class Vector(Location):
    def length(self):
        return math.sqrt(sum(map(lambda x: x ** 2, self)))


class Scene:
    def __init__(self, p, g, is_gravity):
        self.gravity = is_gravity
        self.p = p
        self.g = g


class Object:
    def __init__(self, name, start_position, mass, object_area, start_motion, drag_coefficient, is_static):
        self.name = name
        self.position = start_position
        self.mass = mass
        self.object_area = object_area
        self.motion = start_motion
        self.drag_coefficient = drag_coefficient
        self.is_static = is_static
        self.kinetic_energy = 0.5 * self.mass * (self.motion.length() ** 2)
        self.pulse = self.mass * self.motion.length()
        self.resultant_force = Vector([0, 0])

    def Run(self, forces, time):
        if not self.is_static:

            self.resultant_force = forces[0]
            for value in forces[1:]:
                self.resultant_force = self.resultant_force + value

            acceleration = Vector(self.resultant_force)
            for counter, value in enumerate(acceleration):
                acceleration[counter] = value / self.mass

            for direction_number, direction in enumerate(self.position):
                self.position[direction_number] = self.position[direction_number] + (
                            self.motion[direction_number] * time) + ((acceleration[direction_number] * (time ** 2)) / 2)
                self.motion[direction_number] = self.motion[direction_number] + (acceleration[direction_number] * time)

            self.kinetic_energy = 0.5 * self.mass * (self.motion.length() ** 2)
            self.pulse = self.mass * self.motion.length()

    def Get(self):
        return self.name, self.position


class Physics:

    def __init__(self):
        self.objects = []
        self.scene = Scene(0, 0, True)

    def NewObject(self, this_object):
        if self.objects:
            for i in self.objects:
                if i.name == this_object.name:
                    return 0
        self.objects.append(this_object)
        return 1

    def SetScene(self, p, g, gravity):
        self.scene = Scene(p, g, gravity)

    def Run(self, time, accuracy, save_step_answer):
        out = []
        step_out = []
        for nowTime in range(int(accuracy * time)):
            if self.objects:
                step_out.clear()
                for o in self.objects:
                    step_out.append(o.Get())

                    F = [emptyVector(len(o.position))]
                    if self.scene.p != 0:
                        module_F = o.drag_coefficient * o.object_area * self.scene.p * (o.motion.length() ** 2) * 0.5
                        F.append(-toVector(module_F, o.motion))
                    if self.scene.g != emptyVector(len(self.scene.g)):
                        F.append(self.scene.g * o.mass)
                    if self.scene.gravity and len(self.objects) > 1:
                        for other_o in self.objects:
                            if other_o.name != o.name:
                                distance = Vector(other_o.position - o.position)
                                Ft = (0.0000000000667430151515 * o.mass * other_o.mass) / (distance.length() ** 2)
                                F.append(toVector(Ft, distance))
                    o.Run(F, 1 / accuracy)
                if save_step_answer:
                    out.append((copy.deepcopy(step_out), nowTime/accuracy))
            else:
                return "No objects"
        out.append((copy.deepcopy(step_out), nowTime/accuracy))
        return out

import weakref
from abc import ABCMeta, abstractmethod

import cv2
import numpy as np

# Initiate constance
_cache = weakref.WeakValueDictionary()
scale_by_definition = 0.5
frequency = 400


def set_const(sc=0.5, freq=400):
    global scale_by_definition, frequency
    scale_by_definition = sc
    frequency = freq


def randint(low, high):
    if low == 0 and high == 0:
        return 1
    else:
        return np.random.randint(low, high)


class visual_object(metaclass=ABCMeta):
    def __init__(self):
        self.obj = None
        self.shift = [0, 0]

    @abstractmethod
    def draw(self, image, shift):
        pass


class conveyor(visual_object, metaclass=ABCMeta):
    def __init__(self):
        super().__init__()
        self.coord_by_y = None
        self.length = None
        self.delta = None

    @abstractmethod
    def add(self):
        pass

    @abstractmethod
    def remove_by_distance(self):
        pass

    @abstractmethod
    def fit_for_rendering(self):
        pass

    @abstractmethod
    def update(self):
        pass


class geometric_object:
    def __init__(self, type_object=None, dx=None, dy=None, dr=None):
        self.type_object = type_object
        self.dx = dx
        self.dy = dy
        self.dr = dr
        raise RuntimeError('Call function "rect", "square" or "circle"')

    @classmethod
    def rect(cls, x, y):
        global scale_by_definition, frequency
        if f'rect {x} {y}' not in _cache:
            obj = super().__new__(cls)
            obj.type_object = 'rectangle'
            obj.dx = x
            obj.dy = y
            _cache[f'rect {x} {y}'] = obj
        else:
            obj = _cache[f'rect {x} {y}']
        return obj

    @classmethod
    def square(cls, a):
        global scale_by_definition, frequency
        if f'sqr {a}' not in _cache:
            obj = super().__new__(cls)
            obj.type_object = 'rectangle'
            obj.dx = a
            obj.dy = a
            _cache[f'sqr {a}'] = obj
        else:
            obj = _cache[f'sqr {a}']
        return obj

    @classmethod
    def circle(cls, r):
        global scale_by_definition, frequency
        if f'circle {r}' not in _cache:
            obj = super().__new__(cls)
            obj.type_object = 'circle'
            obj.dr = r
            _cache[f'circle {r}'] = obj
        else:
            obj = _cache[f'circle {r}']
        return obj

    def draw_object(self, image, x, y, color=(125, 125, 0)):
        if self.type_object == 'rectangle':
            cv2.rectangle(image,
                          (int((x - self.dx / 2) * scale_by_definition), int((y - self.dy / 2) * scale_by_definition)),
                          (int((x + self.dx / 2) * scale_by_definition), int((y + self.dy / 2) * scale_by_definition)),
                          color, 1)
        elif self.type_object == 'circle':
            cv2.circle(image, (int(x * scale_by_definition), int(y * scale_by_definition)),
                       int(self.dr * scale_by_definition), color, 2)

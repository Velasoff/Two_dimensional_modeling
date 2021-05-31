from abc import ABC

from conveyors import product_conveyor, tray_conveyor
from handler import handler_for_cv
from meta import *
from robot import robot


class decorate_object(visual_object, ABC):
    def __init__(self, obj: visual_object, init_point: tuple = None, center_point: tuple = None,
                 border=False):
        """
        :param obj: Object for decoration
        :param init_point: top-right creation point
        :param center_point: center creation point
        :param border: whether to build the boundaries of the conveyor
        """
        super().__init__()
        if init_point is not None:
            self.init_point = init_point
            self.center_point = None
        elif center_point is not None:
            self.center_point = center_point
            self.init_point = None
        else:
            raise RuntimeError('The reference point must be indicated')
        self.obj = obj
        self.border = border
        self.number = None
        self.rect: geometric_object = None
        self.selected = False

    # raise(RuntimeError('Call classes "decorate_conveyor" or "decorate_robot"'))

    def draw(self, image, shift=(0, 0), **kwargs):
        self.obj.draw(image, shift)
        if not self.selected:
            cv2.line(image,
                     tuple([int((i + shift[ind]) * scale_by_definition) for ind, i in enumerate(self.center_point)]),
                     (int((self.center_point[0] + 20 + shift[0]) * scale_by_definition),
                      int((self.center_point[1] + shift[1]) * scale_by_definition)),
                     (0, 0, 225), 1)
            cv2.line(image,
                     tuple([int((i + shift[ind]) * scale_by_definition) for ind, i in enumerate(self.center_point)]),
                     (int((self.center_point[0] + shift[0]) * scale_by_definition),
                      int((self.center_point[1] - 20 + shift[1]) * scale_by_definition)),
                     (225, 0, 0), 1)
        else:
            cv2.line(image,
                     tuple([int((i + shift[ind]) * scale_by_definition) for ind, i in enumerate(self.center_point)]),
                     (int((self.center_point[0] + 30 + shift[0]) * scale_by_definition),
                      int((self.center_point[1] + shift[1]) * scale_by_definition)),
                     (0, 0, 225), 2)
            cv2.line(image,
                     tuple([int((i + shift[ind]) * scale_by_definition) for ind, i in enumerate(self.center_point)]),
                     (int((self.center_point[0] + shift[0]) * scale_by_definition),
                      int((self.center_point[1] - 30 + shift[1]) * scale_by_definition)),
                     (225, 0, 0), 2)
        if self.border:
            self.rect.draw_object(image, self.center_point[0] + shift[0], self.center_point[1] + shift[1], (0, 225, 0))


class decorate_conveyor(decorate_object, ABC):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if not isinstance(self.obj, conveyor):
            raise (TypeError('This class must be passed conveyor object!'))
        self._redefine_points()
        self.obj.shift = self.init_point
        self.obj.fit_for_rendering()
        self.add = self.obj.add
        self.remove_by_distance = self.obj.remove_by_distance
        if self.border:
            xy = [int(2 * (i - j)) for i, j in zip(self.center_point, self.init_point)]
            self.rect = geometric_object.rect(xy[0], xy[1])

    def _redefine_points(self):
        if self.center_point is None:
            center_by_x = self.obj.length // 2 + self.init_point[0]
            center_by_y = self.obj.delta * (len(self.obj.coord_by_y) + 1) // 2 + self.init_point[1]
            self.center_point = [center_by_x, center_by_y]
        else:
            center_by_x = self.obj.length // 2
            center_by_y = self.obj.delta * (len(self.obj.coord_by_y) + 1) // 2
            self.init_point = [self.center_point[0] - center_by_x, self.center_point[1] - center_by_y]

    def update(self, selected):
        self.obj.update(selected=selected)
        if self.init_point is None:
            self._redefine_points()
            self.obj.shift = self.init_point


class decorate_robot(decorate_object, ABC):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if not isinstance(self.obj, robot):
            raise (TypeError('This class must be passed robot object!'))
        if self.center_point is None:
            self.center_point = [self.init_point[0] + self.obj.r, self.init_point[1] + self.obj.r]
        else:
            self.init_point = [self.center_point[0] - self.obj.r, self.center_point[1] - self.obj.r]
        self.obj.center_point = np.array(self.center_point, dtype=int)
        self.obj.float_pos = np.array(self.center_point)
        # self.update = self.obj.update

        if self.border:
            self.border = False
            xy = [int(2 * (i - j)) for i, j in zip(self.center_point, self.init_point)]
            self.rect = geometric_object.rect(xy[0], xy[1])
            raise (Warning('"border" is not provided for this class'))

    def update(self, conveyors, selected=False):
        self.obj.center_point = np.array(self.center_point, dtype=int)
        self.obj.update(conveyors, selected=selected)


class two_dim_world:
    def __init__(self, image_definition, delay_visualize=5, decorated=False):
        """
        :param image_definition: Definition of world image
        :param delay_visualize: Image refresh period
        """
        global scale_by_definition, frequency
        self.conveyors = []
        self.robots = []
        self.image = np.zeros((image_definition[0], image_definition[1], 3), np.uint8)
        self.visualize_counter = 0
        self.delay_visualize = delay_visualize
        self.shift = (0, 0)
        self.decorated = decorated
        self.objects_counter = 0
        self.selected = False
        print(f"Resolution scale and frequency are respectively equal: {scale_by_definition} and {frequency} Hz. "
              f"To change them, call the set_const() function.")

        cv2.namedWindow("2D Modeling")
        self.handler = handler_for_cv(self)

    def _check_n_replace(self, obj):
        if self.decorated:
            if not isinstance(obj, decorate_object):
                if isinstance(obj, conveyor):
                    init_point = [0, 0]
                    obj = decorate_conveyor(obj, init_point=init_point)
                    obj.number = self.objects_counter
                    self.objects_counter += 1
                    return obj
                elif isinstance(obj, robot):
                    obj = decorate_robot(obj, center_point=obj.center_point)
                    obj.number = self.objects_counter
                    self.objects_counter += 1
                    return obj
            else:
                obj.number = self.objects_counter
                self.objects_counter += 1
                return obj
        else:
            return obj

    def add_product_conveyor(self, conv: product_conveyor or decorate_object):
        conv = self._check_n_replace(conv)
        self.conveyors.append(conv)

    def add_tray_conveyor(self, conv: tray_conveyor or decorate_object):
        conv = self._check_n_replace(conv)
        self.conveyors.append(conv)

    def add_robot(self, rob: robot or decorate_object):
        rob = self._check_n_replace(rob)
        self.robots.append(rob)

    def update_world(self):
        for conv in self.conveyors:
            conv.update(self.selected)
            conv.add()
            conv.remove_by_distance()
        for rob in self.robots:
            rob.update(self.conveyors, self.selected)

    def update_shift(self, shift):
        self.shift -= shift

    def visualize(self):
        if self.visualize_counter % self.delay_visualize == 0:
            image = self.image.copy()
            for rob in self.robots:
                rob.draw(image, shift=self.shift)
            for conv in self.conveyors:
                conv.draw(image, shift=self.shift)
            cv2.imshow("2D Modeling", image)
            cv2.waitKey(self.delay_visualize * 1000 // frequency)
        self.handler.update()

        # 	print(win32gui.GetCursorInfo(), self.win_cv)
        self.visualize_counter += 1

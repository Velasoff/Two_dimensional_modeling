from meta import *


class product:
    def __init__(self, init_point, shift, belt_value, number, row, column, type_prod: geometric_object):
        global scale_by_definition, frequency
        self.init_point = init_point
        self.shift = shift
        self.belt_value = belt_value
        self.number = number
        self.type_prod = type_prod
        self.row = row
        self.column = column


class tray(product):
    def __init__(self, max_volume, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.max_volume = max_volume
        self.is_full = False
        self.current_volume = 0

    def add_product_to_box(self):
        self.current_volume += 1
        if self.current_volume >= self.max_volume:
            self.is_full = True
        else:
            self.is_full = False


class product_conveyor(conveyor):
    def __init__(self, length: int, length_by_y: int, distance_by_x: int or list, number_rows=1, error_by_x=0,
                 error_by_y=0, velocity=200, type_obj=geometric_object.circle(10)):
        """
        Class of conveyor with product
        :param length: Length of conveyor
        :param length_by_y: Length of conveyor by y coordinates (mm)
        :param distance_by_x: Int or list of x distance for all/each rows (mm)
        :param number_rows: Number of rows
        :param error_by_x: Value of error by x (mm)
        :param error_by_y: Value of error by y (mm)
        :param velocity: Velocity of conveyors (m/s)
        :param type_obj: Graphic prototype for drawing
        """
        super().__init__()
        global scale_by_definition, frequency
        self.delta = length_by_y / (number_rows + 1)
        if number_rows >= 1:
            self.coord_by_y = [self.delta * (i + 1) for i in range(number_rows)]
        else:
            raise Exception("Invalid number of rows.")
        if isinstance(distance_by_x, int):
            self.coord_by_x = [distance_by_x] * number_rows
            self.column = 0
        elif isinstance(distance_by_x, list) and len(distance_by_x) == number_rows:
            self.coord_by_x = distance_by_x
            self.column = None
        else:
            raise Exception('Invalid parameter setting: "distance by x".')
        self.objects = []
        self.number = 0
        self.removed_by_robot = 0
        self.removed_by_distance = 0
        self.error_x = error_by_x
        self.error_y = error_by_y
        self.type_obj = type_obj
        self.velocity = velocity
        self.length = length

    def add_first(self):
        if self.velocity > 0:
            belt_value = 0
        else:
            belt_value = self.length
        for ind, coord_y in enumerate(self.coord_by_y):
            self.objects.append(product([randint(0, 2 * self.error_x),
                                         coord_y + randint(-self.error_y, self.error_y)], self.shift,
                                        belt_value, self.number, ind, self.column, self.type_obj))
            self.number += 1
        if self.column is not None:
            self.column += 1

    def add(self):
        if self.velocity > 0:
            belt_value = 0
        else:
            belt_value = self.length
        for ind, coord_y in enumerate(self.coord_by_y):
            if list(filter(lambda prod: prod.row == ind, self.objects))[-1].belt_value > self.coord_by_x[ind]:
                self.objects.append(product([randint(0, 2 * self.error_x),
                                             coord_y + randint(-self.error_y, self.error_y)], self.shift,
                                            belt_value, self.number, ind, self.column, self.type_obj))
                self.number += 1
        if self.column is not None:
            self.column += 1

    def update(self, selected=False):
        """
        Производит расчёт следующего шага симуляции для продуктов
        """
        if not selected:
            if len(self.objects) == 0:
                self.add_first()
            for a in self.objects:
                a.belt_value += self.velocity / frequency

    def remove_by_robot(self, number_to_remove):
        """
        Удаляет с изображения продукт, если он стал захвачен роботом
        """
        self.objects = list(filter(lambda prod: prod.number != number_to_remove, self.objects))
        self.removed_by_robot += 1

    def remove_by_distance(self):
        """
        Удаляет с изображения продукт по прохождению им заданного значения
        """
        before = len(self.objects)
        self.objects = list(
            filter(lambda prod: ((prod.belt_value + prod.init_point[0]) < self.length) & (prod.init_point[0] + prod.belt_value > 0),
                   self.objects))
        after = len(self.objects)
        self.removed_by_distance += (before - after)
        if before - after != 0:
            return True

    def fit_for_rendering(self):
        init_by_y = self.coord_by_y[0] - self.delta
        if init_by_y != 0:
            self.coord_by_y = [i - init_by_y for i in self.coord_by_y]
        self.add_first()

    def draw(self, image, shift=(0, 0)):
        for prod in self.objects:
            prod.shift = self.shift
            prod.type_prod.draw_object(image, prod.init_point[0] + prod.belt_value + prod.shift[0] + int(shift[0]),
                                       prod.init_point[1] + prod.shift[1] + int(shift[1]))


class tray_conveyor(conveyor):
    def __init__(self, length: int, length_by_y: int, distance_by_x: int, number_rows=1, error_by_x=0,
                 error_by_y=0, velocity=200, max_volume=4, type_obj=geometric_object.square(50)):
        """
        Class of conveyor with product
        :param length: Length of conveyor
        :param length_by_y: Length of conveyor by y coordinates (mm)
        :param distance_by_x: Int of x distance for all rows (mm)
        :param number_rows: Number of tray compartments (points for the arrival of the robot)
        :param error_by_x: Value of error by x (mm)
        :param error_by_y: Value of error by y (mm)
        :param velocity: Velocity of conveyors (m/s)
        :param max_volume: Number of products in one box. Must be a multiple of the number of compartments in the box
        :param type_obj: Graphic prototype for drawing
        """
        super().__init__()
        global scale_by_definition, frequency
        self.delta = length_by_y / (number_rows + 1)
        if number_rows >= 1:
            self.coord_by_y = [self.delta * (i + 1) for i in range(number_rows)]
        else:
            raise ValueError("Invalid number of rows.")
        if isinstance(distance_by_x, int):
            self.coord_by_x = [distance_by_x] * number_rows
        else:
            raise ValueError('Invalid parameter setting: "distance by x".')
        self.objects = []
        self.number = 0
        self.removed_by_robot = 0
        self.removed_by_distance = 0
        self.error_x = error_by_x
        self.error_y = error_by_y
        self.type_obj = type_obj
        self.velocity = velocity
        self.length = length
        self.max_volume = max_volume // number_rows

    def add_first(self):
        if self.velocity > 0:
            belt_value = 0
        else:
            belt_value = self.length
        for ind, coord_y in enumerate(self.coord_by_y):
            self.objects.append(tray(self.max_volume, [randint(0, 2 * self.error_x),
                                                       coord_y + randint(-self.error_y, self.error_y)], self.shift,
                                     belt_value, self.number, ind, 0, self.type_obj))
            self.number += 1

    def add(self):
        if self.velocity > 0:
            belt_value = 0
        else:
            belt_value = self.length
        for ind, coord_y in enumerate(self.coord_by_y):
            if list(filter(lambda prod: prod.row == ind, self.objects))[-1].belt_value > self.coord_by_x[ind]:
                self.objects.append(tray(self.max_volume, [randint(0, 2 * self.error_x),
                                                           coord_y + randint(-self.error_y, self.error_y)], self.shift,
                                         belt_value, self.number, ind, 0, self.type_obj))
                self.number += 1

    def update(self, selected=False):
        """
        Производит расчёт следующего шага симуляции для продуктов
        """
        if not selected:
            if len(self.objects) == 0:
                self.add_first()
            for a in self.objects:
                a.belt_value += self.velocity / frequency

    def remove_by_distance(self):
        """
        Удаляет с изображения продукт по прохождению им заданного значения
        """
        before = len(self.objects)
        self.objects = list(
            filter(lambda prod: ((prod.belt_value + prod.init_point[0]) < self.length) & (prod.init_point[0] + prod.belt_value > 0),
                   self.objects))
        after = len(self.objects)
        self.removed_by_distance += (before - after)
        if before - after != 0:
            return True

    def fit_for_rendering(self):
        init_by_y = self.coord_by_y[0] - self.delta
        if init_by_y != 0:
            self.coord_by_y = [i - init_by_y for i in self.coord_by_y]
        self.add_first()

    def draw(self, image, shift=(0, 0)):
        shift = np.array(shift)
        for prod in self.objects:
            prod.shift = self.shift
            if not prod.is_full:
                prod.type_prod.draw_object(image, prod.init_point[0] + prod.belt_value + prod.shift[0] + int(shift[0]),
                                           prod.init_point[1] + prod.shift[1] + int(shift[1]))
            else:
                prod.type_prod.draw_object(image, prod.init_point[0] + prod.belt_value + prod.shift[0] + int(shift[0]),
                                           prod.init_point[1] + prod.shift[1] + int(shift[1]), color=(0, 255, 0))

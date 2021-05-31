from numba import jit

from conveyors import tray_conveyor, product_conveyor
from meta import *
from motion import motion


class algorithm:
    types = ('left', 'center', 'right', 'draft')

    class simple_algorithm:
        def __init__(self, type_alg: str, draft_fillet: np.ndarray):
            self.objects = []
            if type_alg is 'left':
                self.alg = self.left_selection
            if type_alg is 'center':
                self.alg = self.center_selection
            if type_alg is 'right':
                self.alg = self.right_selection
            if type_alg is 'draft':
                self.alg = self.draft_selection
                if draft_fillet is None:
                    print(UserWarning('You have not chosen a draft fillet, therefore, '
                                      'the algorithm is set with a selection at the right.'))
                    self.alg = self.right_selection
                else:
                    self.fillet = draft_fillet

        def check_objects(self, conv, rob):
            if isinstance(conv, tray_conveyor):
                self.objects = list(filter(lambda box: box.is_full is False, conv.objects))
            else:
                self.objects = conv.objects
            self.objects = list(
                filter(lambda obj: (np.abs(rob.center_point[1] - obj.init_point[1] - obj.shift[1]) < rob.bound_y),
                       self.objects))
            self.objects = list(
                filter(lambda obj: (np.abs(
                    rob.center_point[0] - obj.init_point[0] - obj.shift[0] - obj.belt_value) < rob.bound_x),
                       self.objects))
            return self.alg(rob)

        def left_selection(self, rob):
            min_value = np.inf
            obj = None
            for prod in self.objects:
                if prod.init_point[0] + prod.shift[0] + prod.belt_value < min_value:
                    obj = prod
                    min_value = prod.init_point[0] + prod.shift[0] + prod.belt_value
            return obj

        def center_selection(self, rob):
            min_value = np.inf
            obj = None
            for prod in self.objects:
                if np.abs(rob.center_point[0] - prod.init_point[0] - prod.shift[0] - prod.belt_value) < min_value:
                    obj = prod
                    min_value = np.abs(rob.center_point[0] - prod.init_point[0] - prod.shift[0] - prod.belt_value)
            return obj

        def right_selection(self, rob):
            max_value = 0
            obj = None
            for prod in self.objects:
                if prod.init_point[0] + prod.shift[0] + prod.belt_value > max_value:
                    obj = prod
                    max_value = prod.init_point[0] + prod.shift[0] + prod.belt_value
            return obj

        def draft_selection(self, rob):
            min_value = np.inf
            obj = None
            for prod in self.objects:
                if self.fillet[prod.row, prod.column] is rob.number:
                    if prod.init_point[0] + prod.shift[0] + prod.belt_value < min_value:
                        obj = prod
            return obj

    def __init__(self, alg_box: str = None, alg_prod: str = None, draft_fillet: np.ndarray = None,
                 rows_len: int = None, columns_len: int = None):
        if draft_fillet is not None:
            try:
                draft_fillet.reshape((rows_len, columns_len))
            except Exception:
                raise ValueError("Incorrect draft fillet and/or constants.")
        if alg_box not in self.types:
            print(UserWarning(
                'The algorithm for the selection of boxes is chosen incorrectly. Default value selected: right.'))
            alg_box = self.types[2]
        if alg_prod not in self.types:
            print(UserWarning(
                'The algorithm for the selection of products is chosen incorrectly. Default value selected: right.'))
            alg_prod = self.types[2]
        self.algorithm_for_boxes = self.simple_algorithm(alg_box, draft_fillet)
        self.algorithm_for_products = self.simple_algorithm(alg_prod, draft_fillet)


class robot(visual_object):
    class motion_setting:
        def __init__(self, velocity, acceleration, deceleration, jerk):
            self.velocity = velocity
            self.acceleration = acceleration
            self.deceleration = deceleration
            self.jerk = jerk

        def __call__(self):
            return {'V': self.velocity, 'A': self.acceleration, 'D': self.deceleration, 'J': self.jerk}

    def __init__(self, shift: tuple, x_bound: int, number: np.int0, z_tray: int = 80, z_product: int = 80,
                 d: int = 1130,
                 gripper_rotation: float = None, gripper_open: int = None, motion_setting: motion_setting = None,
                 alg: algorithm = None):
        """
        :param init_point: Coordinates of robot init (mm)
        :param x_bound: Object registration boundary by x relative to central point (mm)
        :param number: Number of robot
        :param z_tray: Tray height relative to work plane (mm)
        :param z_product: Product height relative to work plane (mm)
        :param d: Working area diameter (mm)
        :param gripper_rotation: Grip rotation time (sec)
        :param gripper_open: Grip open/close time (sec)
        :param motion_setting: Motion setting of robot (call self.motion_setting())
        :param alg: Object registration algorithm (call algorithm())
        """
        global scale_by_definition, frequency
        super().__init__()
        if motion_setting is None:
            motion_setting = {'velocity': 10000, 'acceleration': 50000, 'deceleration': 50000, 'jerk': 10 ** 6}
        self.float_pos = np.array(shift)
        self.center_point = np.array(shift, dtype='int')
        self.operations = 0
        self.time = 0
        self.bound_x = x_bound
        self.r = d / 2
        self.bound_y = np.sqrt((self.r ** 2 - x_bound ** 2))
        self.motion = motion()

        if gripper_rotation is not None:
            self.rotation_delay = int(gripper_rotation * frequency)
        else:
            self.rotation_delay = 0

        if gripper_open is not None:
            self.open_delay = int(gripper_open * frequency)
        else:
            self.open_delay = 0

        self.number = number
        self.setting_robot = self.motion_setting(**motion_setting)
        self.buffer = np.zeros(np.int(2 * frequency))  # motion buffer for 2 seconds
        self.buff_velocity = np.zeros(np.int(2 * frequency))
        self.vector = np.zeros(2)  # vector of direction
        self.move_buffer = None
        self.full = False
        self.workaround = 0.0000007
        self.bias = [0, 0]

        self.motion.compute(0, np.linalg.norm(z_tray), 0, 0, **self.setting_robot())
        self.time_to_tray = np.int(self.motion.T * frequency)
        self.motion.compute(0, np.linalg.norm(z_product), 0, 0, **self.setting_robot())
        self.time_to_product = np.int(self.motion.T * frequency)

        self.move_trigger = False
        if alg is not None:
            self.algorithm = alg
        else:
            self.algorithm = algorithm()
        self.conv = None

    def _predict_way(self, start_point, aim_point, velocity):
        first_distance = np.zeros(2)
        second_distance = np.zeros(2)
        while True:
            distance = np.linalg.norm((aim_point + first_distance) - start_point)
            self.motion.compute(0, distance, 0, 0, **self.setting_robot())
            second_distance[0] = self.motion.T * velocity
            if np.linalg.norm(first_distance - second_distance) < 0.1:
                break
            first_distance = second_distance
        return (aim_point + first_distance) - start_point

    def _prepare_vect(self, vect: np.ndarray):
        self.vector = vect / np.linalg.norm(vect)
        print(vect)
        self.motion.compute(0, np.linalg.norm(vect), 0, 0, **self.setting_robot())
        self.move_buffer = np.zeros(np.int(self.motion.T * frequency))
        for i in range(len(self.move_buffer)):
            self.move_buffer[i] = self.motion.position((i + 1) / frequency)
        self.move_buffer = self.move_buffer[1:] - self.move_buffer[:-1]
        self.move_trigger = True
        self._insert_move_in_buffer()

    def _insert_move_in_buffer(self):
        delay = 0
        if not self.full:
            z_delay = self.time_to_product
        else:
            z_delay = self.time_to_tray
        self.buffer[0:len(self.move_buffer)] = self.move_buffer
        delay += len(self.move_buffer)
        if z_delay + len(self.move_buffer) >= self.rotation_delay:
            self.buffer[delay:delay + z_delay] = np.ones(z_delay) * self.workaround
            delay += z_delay
        else:
            self.buffer[delay:self.rotation_delay] = np.ones(self.rotation_delay - delay) * self.workaround
            delay = self.rotation_delay
        if self.open_delay > 0:
            self.buffer[delay:delay + self.open_delay] = np.ones(self.open_delay) * self.workaround
            delay += self.open_delay
        self.buffer[delay:delay + z_delay] = np.ones(z_delay) * self.workaround
        self.move_buffer = None

    def _cyclic_buffer_pop(self):
        self.buffer = np.append(self.buffer[1:], 0)

    def _stop_on_border(self):
        if np.linalg.norm(np.abs(self.float_pos + self.vector * self.buffer[0] - self.center_point)) > self.r:
            print(Warning("Robot can't reach in this point! Point: " +
                          str(self.float_pos + self.vector * self.buffer[0])))
            self.buffer = np.zeros(np.int(2 * frequency))
            self.vector = np.zeros(2)
            self.obj = None

    def _check_move(self):
        if self.buffer[0] == 0:
            self.move_trigger = False
            if self.obj is not None:
                if self.full:
                    self.obj.add_product_to_box()
                    self.obj = None
                    self.full = False
                else:
                    self.conv.remove_by_robot(self.obj.number)
                    self.obj = None
                    self.full = True

    def _prepare_object(self, conveyors):
        velocity = None
        if not self.full and not self.move_trigger:
            for conv in conveyors:
                if isinstance(conv, product_conveyor):
                    self.obj = self.algorithm.algorithm_for_products.check_objects(conv, self)
                    velocity = conv.velocity
                    self.conv = conv
                elif isinstance(conv.obj, product_conveyor):
                    self.obj = self.algorithm.algorithm_for_products.check_objects(conv.obj, self)
                    velocity = conv.obj.velocity
                    self.conv = conv.obj
        if self.full and not self.move_trigger:
            for conv in conveyors:
                if isinstance(conv, tray_conveyor):
                    self.obj = self.algorithm.algorithm_for_boxes.check_objects(conv, self)
                    velocity = conv.velocity
                    self.conv = conv
                elif isinstance(conv.obj, tray_conveyor):
                    self.obj = self.algorithm.algorithm_for_products.check_objects(conv.obj, self)
                    velocity = conv.obj.velocity
                    self.conv = conv.obj
        if self.obj is not None and not self.move_trigger:
            vect = self._predict_way(self.float_pos, np.array(
                [self.obj.init_point[0] + self.obj.shift[0] + self.obj.belt_value,
                 self.obj.init_point[1] + self.obj.shift[1]]), velocity)
            self._prepare_vect(vect)

    @staticmethod
    @jit
    def _compute_current_position(float_pos, vector, buff):
        return float_pos + vector * buff

    def update(self, conveyors: list, selected: bool):
        if not selected:
            self._prepare_object(conveyors)
            self._check_move()
            if self.buffer[0] == self.workaround:
                self.buffer[0] = 0
            self._stop_on_border()
            self.float_pos = self._compute_current_position(self.float_pos, self.vector, self.buffer[0])
            self._cyclic_buffer_pop()
            self.bias = self.float_pos - self.center_point
        else:
            self.buffer = np.zeros(np.int(2 * frequency))
            self.vector = np.zeros(2)
            self.obj = None
            self.float_pos = self.bias + self.center_point

    def draw(self, image, shift=(0, 0)):
        cv2.circle(image, tuple(np.array(self.center_point * scale_by_definition, dtype='int32') +
                                np.array([i * scale_by_definition for i in shift], dtype='int32')),
                   int(self.r * scale_by_definition), (128, 0, 0), 5)
        cv2.circle(image, tuple(np.array(self.float_pos * scale_by_definition, dtype='int32') +
                                np.array([i * scale_by_definition for i in shift], dtype='int32')),
                   int(10 * scale_by_definition), (50, 205, 154), 2)

from robot import algorithm
from two_dim_modeling import *
import random
from win32api import GetSystemMetrics

if __name__ == '__main__':
    set_const(sc=0.5)
    world = two_dim_world((1, 1), 5, True)
    # world.add_product_conveyor(decorate_conveyor(product_conveyor(3000, 700, 200, 2,
    #                                                               type_obj=geometric_object.circle(30)),
    #                                              init_point=(120, 500), border=True))
    # world.add_tray_conveyor(tray_conveyor(3000, 200, 100, 2, max_volume=1))
    # world.add_robot(decorate_robot(robot((1500, 500), 380, 1, alg=algorithm(alg_box=algorithm.types[1])),
    #                                init_point=(120, 500)))
    while True:
        world.update_world()
        world.visualize()
        # if random.random() > 0.999:
        #     world.set_shift(np.random.randint(-200, 200, 2))

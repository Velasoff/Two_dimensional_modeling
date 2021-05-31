import numpy as np
import win32api
import win32con
import win32gui
from win32api import GetSystemMetrics

from meta import scale_by_definition

class handler_for_cv:
    def __init__(self, world):
        self.win_cv = win32gui.FindWindow(None, "2D Modeling")
        win32gui.SetWindowLong(self.win_cv, win32con.GWL_STYLE, 0)
        win32gui.ShowWindow(self.win_cv, win32con.SW_SHOW)
        win32gui.SetWindowPos(self.win_cv, win32con.HWND_TOPMOST,
                              GetSystemMetrics(0) // 7, GetSystemMetrics(1) // 13,
                              GetSystemMetrics(0) // 7 * 5,
                              GetSystemMetrics(1) // 13 * 11,
                              win32con.SWP_FRAMECHANGED)
        self.win_cv2_coord = np.array(win32gui.GetWindowRect(self.win_cv)).reshape((-1, 2))
        print(self.win_cv2_coord)
        self.world = world
        self.current_cursor = np.array((0, 0))
        self.obj = None
        self.prev_click = 2
        self.prev_pos = [0, 0]

    def _on_click(self):
        self.current_cursor = (np.array(win32gui.GetCursorInfo()[2]) - self.win_cv2_coord[0]) / scale_by_definition
        return True if win32api.GetKeyState(win32con.VK_LBUTTON) < 0 else False

    def _capture_objects(self):
        for obj in self.world.conveyors + self.world.robots:
            if self.current_cursor[0] - 20 < obj.center_point[0] + self.world.shift[0] < self.current_cursor[
                0] + 20 and self.current_cursor[1] - 10 < obj.center_point[1] + self.world.shift[1] < \
                    self.current_cursor[1] + 10:
                self.obj = obj
                self.world.selected = True
                self.obj.selected = True
                break
            elif self.win_cv2_coord[0, 0] < win32gui.GetCursorInfo()[2][0] < self.win_cv2_coord[1, 0] and \
                    self.win_cv2_coord[0, 1] < win32gui.GetCursorInfo()[2][1] < self.win_cv2_coord[1, 1]:
                print(win32gui.GetCursorInfo()[2])
                if self.obj is None:
                    self.obj = 'world'
                    self.prev_pos = self.current_cursor

    def _change_parameters(self):
        if self.obj is 'world':
            self.world.update_shift(self.prev_pos - self.current_cursor)
            self.prev_pos = self.current_cursor
        else:
            self.obj.center_point = [self.current_cursor[0] - self.world.shift[0],
                                     self.current_cursor[1] - self.world.shift[1]]
            self.obj.init_point = None

    def _reset_object(self):
            if self.obj is not None and self.obj is not 'world':
                self.obj.selected = False
                self.world.selected = False
            self.obj = None

    def _update_cursor(self):
        if self._on_click():
            if self.prev_click >= 0 and self.obj is None:
                self._capture_objects()
            elif self.obj is not None:
                self._change_parameters()
        else:
            self._reset_object()
        self.prev_click = win32api.GetKeyState(win32con.VK_LBUTTON)

    def update(self):
        self._update_cursor()

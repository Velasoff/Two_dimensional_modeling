import math

from numpy import imag
from numpy import real
from numpy import roots


class motion:
    """
    Класс профиля движения с ограничением рывка
    """

    def __init__(self):
        pass

    def compute(self, p0, pf, v0, vf, V, A, D, J, t0=0.0):
        """
        Вычисление траектории по указанным параметрам
        p0 - начальное положение;
        pf - конечное положение;
        v0 - начальная скорость;
        vf - конечная скорость;
        V - максимальная допустимая скорость;
        A - максимальное допустимое ускорение разгона;
        D - максимальное допустимое ускорение торможения;
        J - максимальный допустимый рывок
        """
        # Запись начальных параметров
        self.p0 = p0
        self.pf = pf
        self.v0 = v0
        self.vf = vf
        self.V = V
        self.A = A
        self.D = D
        self.J = J
        self.t0 = t0
        self.AFP = self.isAFP()

        # Запись внутренних параметров, переход от DFP к AFP при необходимости
        self._p = [None] * 8
        self._v = [None] * 8
        self._t = [None] * 8
        if self.AFP:
            self._V = V
            self._A = A
            self._D = D
            self._J = J
            self._p[0] = p0
            self._p[7] = pf
            self._v[0] = v0
            self._v[7] = vf
        else:
            self._V = V
            self._A = D
            self._D = A
            self._J = J
            self._p[0] = -p0
            self._p[7] = -pf
            self._v[0] = -v0
            self._v[7] = -vf

        self._t[0] = t0
        # Рассчет периодов ускорения, торможения и постоянной скорости профиля
        (x, vp) = self.evaluatePeriods()
        # Изменения ограничений ускорения при необходимости
        # if vp - v0 > 0.25*J*pow(x[0],2):
        if self._A > J * x[0] / 2:
            self._A = 0.5 * J * x[0]
        # if vp - vf > 0.25*J*pow(x[2],2):
        if self._D > J * x[2] / 2:
            self._D = 0.5 * J * x[2]

        # Рассчет временных участков профиля
        self._T = [0.0]
        self._T.append(self._A / self._J)
        self._T.append(x[0] - 2 * self._T[1])
        self._T.append(self._T[1])
        self._T.append(x[1])
        self._T.append(self._D / self._J)
        self._T.append(x[2] - 2 * self._T[5])
        self._T.append(self._T[5])

        for i in range(1, 8):
            self._t[i] = self._t[i - 1] + self._T[i]

        self.T = self._t[7] - self._t[0]

        # Рассчет границ участков профиля
        self.calcIntervalBorderConditions()

    def isAFP(self):
        """
        Возвращает True, если период разгона идет первым.
        False - если период торможения.
        """
        v0 = self.v0
        vf = self.vf
        L = self.pf - self.p0
        if v0 <= vf:
            if vf - v0 <= pow(self.A, 2) / self.J:
                Tm = 2 * math.sqrt((vf - v0) / self.J)
            else:
                Tm = (vf - v0) / self.A + self.A / self.J
        else:
            if v0 - vf <= pow(self.D, 2) / self.J:
                Tm = 2 * math.sqrt((v0 - vf) / self.J)
            else:
                Tm = (v0 - vf) / self.D + self.D / self.J

        Lm = (v0 + vf) / 2 * Tm
        return (v0 <= vf and Lm <= L) or (v0 > vf and Lm < L)

    def position(self, time):
        """
        Возвращает положение профиля в момент времени time
        Выполняет расчет исходя из того, на каком участке профиля
        находится момент времени time.
        """
        if time < self._t[0]:
            p = self._p[0]
        elif time < self._t[1]:
            p = self._p[0] + self._v[0] * (time - self._t[0]) + \
                self._J * pow(time - self._t[0], 3) / 6
        elif time < self._t[2]:
            p = self._p[1] + self._v[1] * (time - self._t[1]) + \
                0.5 * self._A * pow(time - self._t[1], 2)
        elif time < self._t[3]:
            p = self._p[2] + self._v[2] * (time - self._t[2]) + \
                0.5 * self._A * pow(time - self._t[2], 2) - \
                self._J * pow(time - self._t[2], 3) / 6
        elif time < self._t[4]:
            p = self._p[3] + self._v[3] * (time - self._t[3])
        elif time < self._t[5]:
            p = self._p[4] + self._v[4] * (time - self._t[4]) - \
                self._J * pow(time - self._t[4], 3) / 6
        elif time < self._t[6]:
            p = self._p[5] + self._v[5] * (time - self._t[5]) - \
                0.5 * self._D * pow(time - self._t[5], 2)
        elif time < self._t[7]:
            p = self._p[6] + self._v[6] * (time - self._t[6]) - \
                0.5 * self._D * pow(time - self._t[6], 2) + \
                self._J * pow(time - self._t[6], 3) / 6
        else:
            p = self._p[7] + self._v[7] * (time - self._t[7])

        if self.AFP:
            return p
        else:
            return -p

    def velocity(self, time):
        """
        Возвращает скорость профиля в момент времени time
        Выполняет расчет исходя из того, на каком участке профиля
        находится момент времени time.
        """
        if time < self._t[0]:
            v = self._v[0]
        elif time < self._t[1]:
            v = self._v[0] + 0.5 * self._J * pow(time - self._t[0], 2)
        elif time < self._t[2]:
            v = self._v[1] + self._A * (time - self._t[1])
        elif time < self._t[3]:
            v = self._v[2] + self._A * (time - self._t[2]) \
                - 0.5 * self._J * pow(time - self._t[2], 2)
        elif time < self._t[4]:
            v = self._v[3]
        elif time < self._t[5]:
            v = self._v[4] - 0.5 * self._J * pow(time - self._t[4], 2)
        elif time < self._t[6]:
            v = self._v[5] - self._D * (time - self._t[5])
        elif time < self._t[7]:
            v = self._v[6] - self._D * (time - self._t[6]) \
                + 0.5 * self._J * pow(time - self._t[6], 2)
        else:
            v = self._v[7]

        if self.AFP:
            return v
        else:
            return -v

    def acceleration(self, time):
        """
        Возвращает ускорение профиля в момент времени time.
        Выполняет расчет исходя из того, на каком участке профиля
        находится момент времени time.
        """
        if time < self._t[0]:
            a = 0.0
        elif time < self._t[1]:
            a = self._J * (time - self._t[0])
        elif time < self._t[2]:
            a = self._A
        elif time < self._t[3]:
            a = self._A - self._J * (time - self._t[2])
        elif time < self._t[4]:
            a = 0.0
        elif time < self._t[5]:
            a = -self._J * (time - self._t[4])
        elif time < self._t[6]:
            a = -self._D
        elif time < self._t[7]:
            a = -self._D + self._J * (time - self._t[6])
        else:
            a = 0.0

        if self.AFP:
            return a
        else:
            return -a

    def calcIntervalBorderConditions(self):
        """
        Внутренняя функция. Вычисляет значения скорости
        и положения на границах временных участков профиля.
        """
        T = self._T

        self._v[1] = self._v[0] + 0.5 * self._J * pow(T[1], 2)
        self._v[2] = self._v[1] + self._A * T[2]
        self._v[3] = self._v[2] + self._A * T[3] - 0.5 * self._J * pow(T[3], 2)
        self._v[4] = self._v[3]
        self._v[5] = self._v[4] - 0.5 * self._J * pow(T[5], 2)
        self._v[6] = self._v[5] - self._D * T[6]

        self._p[1] = self._p[0] + self._v[0] * T[1] + self._J * pow(T[1], 3) / 6
        self._p[2] = self._p[1] + self._v[1] * T[2] + 0.5 * self._A * pow(T[2], 2)
        self._p[3] = self._p[2] + self._v[2] * T[3] + 0.5 * self._A * pow(T[3], 2) \
                     - self._J * pow(T[3], 3) / 6
        self._p[4] = self._p[3] + self._v[3] * T[4]
        self._p[5] = self._p[4] + self._v[4] * T[5] - self._J * pow(T[5], 3) / 6
        self._p[6] = self._p[5] + self._v[5] * T[6] - 0.5 * self._D * pow(T[6], 2)

    def evaluatePeriods(self):
        """
        Внутренняя функция. Определяет периоды профиля.
        Подряд проверяются все условия, пока не будет найдено подходящее.
        Возвращает кортеж вида ((xa,xc,xd),vp),
        xa - период разгона
        xc - период постоянной скорости
        xd - период торможения
        vp - пиковая скорость
        """
        V = self._V
        A = self._A
        D = self._D
        J = self._J
        v0 = self._v[0]
        vf = self._v[7]
        L = self._p[7] - self._p[0]

        # Варианты без участка с постоянной скоростью
        def caseNoCVelCAccCDec():
            # Есть участки с постоянным разгоном и торможением, нет постоянной скорости
            a = A * (A / D + 1)
            b = 1 / (J * D) * (A + D) * (A * D - 2 * pow(A, 2) + 2 * v0 * J)
            c = -2 * L - 1 / D * (v0 + vf - pow(A, 2) / J) * (vf - v0 + (pow(A, 2) - pow(D, 2)) / J)
            x = roots([a, b, c])
            for xi in x:
                if imag(xi) == 0 and xi >= 2 * A / J:
                    xa = real(xi)
                    xd = (v0 - vf - pow(A, 2) / J + pow(D, 2) / J + A * xa) / D
                    if xd >= 2 * D / J:
                        vp = v0 - pow(A, 2) / J + A * xa
                        return ((xa, 0.0, xd), vp)
                    else:
                        continue
                else:
                    continue
            else:
                return

        def caseNoCVelNoCAccCDec():
            # Нет постоянного разгона, есть постоянное торможение, нет постоянной скорости
            a = pow(J, 2) / (16 * D)
            b = J / 4
            c = (2 * J * v0 / D + D) / 4
            d = 2 * v0
            e = -2 * L + (v0 + vf) * (v0 - vf + pow(D, 2) / J) / D
            x = roots([a, b, c, d, e])
            for xi in x:
                if imag(xi) == 0 and xi < 2 * A / J and xi >= 0:
                    xa = real(xi)
                    xd = (v0 + J / 4 * pow(xa, 2) - vf + pow(D, 2) / J) / D
                    if xd >= 2 * D / J:
                        vp = vf - pow(D, 2) / J + D * xd
                        return ((xa, 0.0, xd), vp)
                    else:
                        continue
                else:
                    continue
            else:
                return

        def caseNoCVelCAccNoCDecc():
            # Есть постоянный разгон, нет постоянного торможения, нет постоянной скорости
            a = pow(J, 2) / (16 * A)
            b = J / 4
            c = (2 * J * vf / A + A) / 4
            d = 2 * vf
            e = -2 * L + (v0 + vf) * (vf - v0 + pow(A, 2) / J) / A
            x = roots([a, b, c, d, e])
            for xi in x:
                if imag(xi) == 0 and xi < 2 * D / J and xi >= 0:
                    xd = real(xi)
                    xa = (vf + J / 4 * pow(xd, 2) - v0 + pow(A, 2) / J) / A
                    if xa >= 2 * A / J:
                        vp = v0 - pow(A, 2) / J + A * xa
                        return ((xa, 0.0, xd), vp)
                    else:
                        continue
                else:
                    continue
            else:
                return

        def caseNoCVelNoCAccNoCDecc():
            # Нет постоянного разгона и торможения, нет постоянной скорости
            a = (vf - v0) * J / 4
            b = J * L
            c = -pow(vf - v0, 2)
            d = 8 * v0 * L
            e = -4 * (pow(L, 2) + pow(v0 + vf, 2) * (vf - v0) / J)
            x = roots([a, b, c, d, e])
            for xi in x:
                if imag(xi) == 0 and xi < 2 * A / J and xi >= 0:
                    xa = real(xi)
                    a2 = J / 4
                    b2 = 0.0
                    c2 = vf - v0 - J / 4 * pow(xa, 2)
                    x2 = roots([a2, b2, c2])
                    for xj in x2:
                        if imag(xj) == 0 and xj < 2 * D / J and xj >= 0:
                            xd = real(xj)
                            vp = v0 + J / 4 * pow(xa, 2)
                            return ((xa, 0.0, xd), vp)
                        else:
                            continue
                else:
                    continue
            else:
                return

        # Варианты с участком постоянной скорости
        def calcXC(xa, xd):
            # Вычисляет период движения с постоянной скоростью
            xc = (2 * L - (v0 + V) * xa - (V + vf) * xd) / 2 / V
            if xc >= 0:
                return xc
            else:
                return

        def caseCvelCAccCDec():
            # Есть постоянная скорость, постоянный разгон, постоянное торможение
            xa = (V - v0) / A + A / J
            if xa <= 2 * A / J:
                return
            xd = (V - vf) / D + D / J
            if xd <= 2 * D / J:
                return
            xc = calcXC(xa, xd)
            if xc is None:
                return
            else:
                return ((xa, xc, xd), V)

        def caseCvelNoCAccCDec():
            # Есть постоянная скорость,нет постоянного разгона,есть постоянное торможение
            xa = 2 * math.sqrt((V - v0) / J)
            if xa >= 2 * A / J or xa < 0:
                return
            xd = (V - vf) / D + D / J
            if xd <= 2 * D / J:
                return
            xc = calcXC(xa, xd)
            if xc is None:
                return
            else:
                return ((xa, xc, xd), V)

        def caseCvelCAccNoCDec():
            # Есть постоянная скорость, постоянный разгон,нет постоянного торможения
            xa = (V - v0) / A + A / J
            if xa <= 2 * A / J:
                return
            xd = 2 * math.sqrt((V - vf) / J)
            if xd >= 2 * D / J or xd < 0:
                return
            xc = calcXC(xa, xd)
            if xc is None:
                return
            else:
                return ((xa, xc, xd), V)

        def caseCvelNoCAccNoCDec():
            # Есть постоянная скорость,нет постоянного разгона и торможения
            xa = 2 * math.sqrt((V - v0) / J)
            if xa >= 2 * A / J or xa < 0:
                return
            xd = 2 * math.sqrt((V - vf) / J)
            if xd >= 2 * D / J or xd < 0:
                return
            xc = calcXC(xa, xd)
            if xc is None:
                return
            else:
                return ((xa, xc, xd), V)

        cases = [caseCvelCAccCDec, caseCvelCAccNoCDec, caseCvelNoCAccCDec, \
                 caseCvelNoCAccNoCDec, caseNoCVelCAccCDec, caseNoCVelCAccNoCDecc, \
                 caseNoCVelNoCAccCDec, caseNoCVelNoCAccNoCDecc]

        for case in cases:
            res = case()
            if res is not None:
                return res
            else:
                continue
        else:
            print("Не удалось вычислить периоды")
            return

    def shiftProfile(self, posShift, timeShift):
        """
        Сдвигает профиль по положению или времени без изменения его вида
        posShift - сдвиг по позиции
        timeShift - сдвиг по времени
        """
        if timeShift != 0.0:
            self.t0 = self.t0 + timeShift
            for i in range(len(self._t)):
                self._t[i] = self._t[i] + timeShift

        if posShift != 0.0:
            for i in range(len(self._p)):
                self._p[i] = self._p[i] + posShift

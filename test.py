class A:
	def __init__(self):
		self.a = 2
		self.c = 1

	def foo(self):
		print(self.a)


class B:
	def __init__(self):
		self.b = 4
		self.c = 5

	def bar(self):
		print(self.b)

	def foo(self):
		print(self.c)


class C(A, B):
	def bar(self):
		print(self.a)


c = C()
c.bar()
c.foo()

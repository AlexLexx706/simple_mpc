from casadi import *

opti = Opti()
x = opti.variable()
y = opti.variable()

opti.minimize(  (y-x**2)**2   )
opti.subject_to( x**2+y**2==1 )
opti.subject_to(       x+y>=1 )

opti.solver('ipopt')
f = opti.to_function("f",[x, y],[x])

opts = dict(main=True, mex=True)
f.generate('gen.c', opts)
print(open('gen.c', 'r').read())

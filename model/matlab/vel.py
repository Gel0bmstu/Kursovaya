import math

f = open('R.txt', 'r')
v = open('V.txt', 'w')

V = []

G = 6.67408e-11 # [m^3/kg/s^2]
M = 5.972e24    # [kg]

for line in f:
    r = float(line) * 1000
    vel = math.sqrt(M * G / r)
    v.write(str('{}\n'.format(vel)))
    V.append(vel)

print(V)

f.close()
v.close()

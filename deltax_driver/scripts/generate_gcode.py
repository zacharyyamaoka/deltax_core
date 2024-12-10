
Xs = [0]
Ys = [0]
Ws = [-40, 0, 40]
Us = [-45, -22, 22, 45]
v0 = -25
Vs = [v0+25, v0, v0-10]

X = 0
Y = 380
Z = -880

W = []

for W in Ws:
    for V in Vs:
        print(f"G0 X{X} Y{Y} Z{Z} W{W} U{0} V{V} S0 E0 F3000 A100")
        # print(f"G4 P1000")

    for U in Us:
        print(f"G0 X{X} Y{Y} Z{Z} W{W} U{U} V{v0} S0 E0 F3000 A100")
        # print(f"G4 P1000")


print(f"G0 X0 Y0 Z-800 W0 U0 V{v0} S0 E0 F3000 A100")
print("G28")
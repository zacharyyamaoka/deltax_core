
Xs = [0]
Ys = [0]
Ws = [160, 180, 200]
Us = [-30, -15, 15]
v0 = -15
Vs = [-25, v0, 0]

X = -75
Y = 290
Z = -880

W = []

for W in Ws:
    for V in Vs:
        print(f"G0 X{X} Y{Y} Z{Z} W{W} U{0} V{V} S0 E0 F3000 A100")
        # print(f"G4 P1000")

    for U in Us:
        print(f"G0 X{X} Y{Y} Z{Z} W{W} U{U} V{v0} S0 E0 F3000 A100")
        # print(f"G4 P1000")


# Define Stations and primary axis
# Xs = [-32, 0, 32]
# Ys = [398, 380, 398]
Xs = [-61, 0, 61]
Ys = [395, 340, 395]
Ws = [-60, 0, 60]

# Define Hemisphere of rotation
Us = [[30, 60],[45, 22, -22, -45],[-30, -60]]
v0 = -25
Vs = [v0+25, v0+10, v0]

X = 0
Y = 380
Z = -880


for i in range(3):
    W = Ws[i]
    X = Xs[i]
    Y = Ys[i]

    for V in Vs:
        print(f"G0 X{X} Y{Y} Z{Z} W{W} U{0} V{V} S0 E0 F3000 A100")
        # print(f"G4 P1000")

    for U in Us[i]:
        print(f"G0 X{X} Y{Y} Z{Z} W{W} U{U} V{v0} S0 E0 F3000 A100")
        # print(f"G4 P1000")


print(f"G0 X0 Y0 Z-800 W0 U0 V{v0} S0 E0 F3000 A100")
print("G28")
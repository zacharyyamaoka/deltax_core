
# MAXIMAL ROTATION
# Define Stations and primary axis
Xs = [-61, 0, 61]
Ys = [395, 340, 395]
Zs = [-880, -860, -880]
Ws = [-60, 0, 60]
# Define Hemisphere of rotation
Us = [[30, 60],[45, 22.5, -22.5, -45],[-30, -60]]
Vs = [45, 15, -15]



for i in range(3):
    W = Ws[i]
    X = Xs[i]
    Y = Ys[i]
    Z = Zs[i]

    for V in Vs:
        print(f"G0 X{X} Y{Y} Z{Z+V} W{W} U{0} V{V} S0 E0 F3000 A100") # Z + V makes Z axis move up and down to counteract angle
        # print(f"G4 P1000")

    for U in Us[i]:
        print(f"G0 X{X} Y{Y} Z{Z} W{W} U{U} V{0} S0 E0 F3000 A100")
        # print(f"G4 P1000")


print(f"G0 X0 Y0 Z-800 W0 U0 V0 S0 E0 F3000 A100")
print("G28")
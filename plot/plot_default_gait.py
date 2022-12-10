import matplotlib.pyplot as plt
import numpy as np

'''
Plots Pranav's hand-tuned gait

'''


fig, (leg1, leg2, leg3, leg4) = plt.subplots(4)
leg1.set_title("Hand-Tuned HSA Gait")

width = 20
x = np.array([0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000])
servo_0 = [90, -90, -90, -90, -90, -90, -90, 90, 90, 90, 0]
servo_1 = [-90, 90, 90, 90, 90, 90, 90, -90, -90, -90, 0]
servo_2 = [90, -90, -90, 90, 90, 90, 90, 90, 90, 90, 0]
servo_3 = [-90, 90, 90, -90, -90, -90, -90, -90, -90, -90, 0]

servo_4 = [90, 90, 90, 90, -90, -90, -90, -90, -90, -90, 0]
servo_5 = [-90, -90, -90, -90, 90, 90, 90, 90, 90, 90, 0]
servo_6 = [90, 90, 90, 90, -90, -90, -90, -90, -90, 90, 0]
servo_7 = [-90, -90, -90, -90, 90, 90, 90, 90, 90, -90, 0]

servo_8 = [90, -90, -90, -90, -90, -90, -90, 90, 90, 90, 0]
servo_9 = [-90, 90, 90, 90, 90, 90, 90, -90, -90, -90, 0]
servo_10 = [90, -90, 90, 90, 90, 90, 90, 90, 90, 90, 0]
servo_11 = [-90, 90, -90, -90, -90, -90, -90, -90, -90, -90, 0]

servo_12 = [90, 90, 90, 90, 90, -90, -90, 90, 90, 90, 0]
servo_13 = [-90, -90, -90, -90, -90, 90, 90, -90, -90, -90, 0]
servo_14 = [90, 90, 90, 90, 90, -90, 90, 90, 90, 90, 0]
servo_15 = [-90, -90, -90, -90, -90, 90, -90, -90, -90, -90, 0]

leg1.bar(x-30, servo_0, width, color = "red")
leg1.bar(x-10, servo_1, width, color = "blue")
leg1.bar(x+10, servo_2, width, color = "red")
leg1.bar(x+30, servo_3, width, color = "blue")

leg2.bar(x-30, servo_4, width, color = "red")
leg2.bar(x-10, servo_5, width, color = "blue")
leg2.bar(x+10, servo_6, width, color = "red")
leg2.bar(x+30, servo_7, width, color = "blue")

leg3.bar(x-30, servo_8, width, color = "red")
leg3.bar(x-10, servo_9, width, color = "blue")
leg3.bar(x+10, servo_10, width, color = "red")
leg3.bar(x+30, servo_11, width, color = "blue")

leg4.bar(x-30, servo_12, width, color = "red")
leg4.bar(x-10, servo_13, width, color = "blue")
leg4.bar(x+10, servo_14, width, color = "red")
leg4.bar(x+30, servo_15, width, color = "blue")
leg4.set_xlabel("Time (ms)")
leg4.set_ylabel("Servo Angle (deg)")


fig.tight_layout()
plt.show()
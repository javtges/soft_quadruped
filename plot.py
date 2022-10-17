import numpy as np
import matplotlib.pyplot as plt
import csv

reward = []
x_values = []
data = []


# '../trial_220930_134147_opt_start'
# trial_220930_135226_rand_start
with open('../trial_220930_134147_opt_start') as f:
    reader = csv.reader(f, delimiter=',')
    for row in reader:
        # data.append(row)
        reward.append(row[16])
    
x_values = np.arange(len(reward))
reward = [float(x) for x in reward]
print(reward)
plt.plot(x_values, reward)
plt.yticks(np.arange(0, 0.01, step=0.001))
plt.title('Hand-Tuned Gait Starting Vector')
plt.xlabel("Trial Number")
plt.ylabel("Speed in m/s")
plt.show()
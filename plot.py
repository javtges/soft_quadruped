import numpy as np
import matplotlib.pyplot as plt
import csv

reward = []
x_values = []
data = []


'''



'''


# '../trial_220930_134147_opt_start'
# trial_220930_135226_rand_start
# with open('../trial_220930_134147_opt_start') as f:
#     reader = csv.reader(f, delimiter=',')
#     for row in reader:
#         # data.append(row)
#         reward.append(row[16])


# reward = [1.2710180966319349,1.6855985432324943,1.294745653883418,1.2894853166763618,1.3767319388230006,
# 1.2847467181323577,1.3479708784455608,1.7991953958307487,1.301582008799117,-9.294636177507812,2.731729384165109,
# 2.200361534130714,1.518818053783918,0.3615341843762892,1.8039964221143736,4.7792369709228915,5.250834539789632,
# 3.6700289805060375,3.604720732467295,14.288503139386188,6.466682112552191,12.212760380508893,1.2389021376076161,
# 15.299518970632015,1.8459098807442724,5.324042845456088,17.31256873553398,1.2772275242038653,3.3986512687342425,
# 5.1396203926626285,1.5307981269031836,7.122630721736192,7.080145755442392,1.198252020586312,16.602873269419458]    

reward = [-0.07267226158147178, -0.09097002314138847, 0.022578266299220107, 0.08228400048336532,
          0.345019242670644, -0.1600815864018138, -0.08581146108897186, -0.05178142212562897,
        -0.4805989214381423, 0.0583695536792862, -0.2736547987152315, -0.016400559289687033,
        -0.0864191193011028, -0.12282004075716602, 0.25610876664275695, 0.08734211697820232]


x_values = np.arange(len(reward))
reward = [float(x) for x in reward]
print(reward)
plt.plot(x_values, reward)
# plt.yticks(np.arange(0, 0.01, step=0.001))
plt.title('Hand-Tuned Gait Starting Vector')
plt.xlabel("Trial Number")
plt.ylabel("Speed in m/s")
plt.show()
import numpy as np
from randompoly import *
from LPsigma import *

initformation = np.array([[0,1,1],[0.6,0.75,1],[0.6,-0.75,1],[0,-1,1],[-0.6,-0.75,1],[-0.6,0.75,1]]).T
rad = 0.2
min_state = [0,0,0,0,0]
max_state = [0,0,360,2,2]
Sxmax2 = max_state[3]**2
Symax2 = max_state[4]**2
new = np.array(min_state) + np.random.rand(5) * (np.array(max_state) - np.array(min_state))
S=generate_random_scaling(initformation,rad,Sxmax2,Symax2)
print(S)
new[3] = np.sqrt(S[0])
new[4] = np.sqrt(S[1])
print(new)
'''
Quote from:
https://towardsdatascience.com/introduction-to-augmented-random-search-d8d7b55309bd

Let 𝛎 a positive constant < 1
Let 𝝰 be the learning rate
Let N the number of perturbations
Let 𝜃 a (p x n) matrix representing the parameters of the policy 𝜋
Let 𝜹i a (p x n) matrix representing the ith perturbation
1. While end condition not satisfied do:
2. Generate N perturbations 𝜹 from a normal distribution
3. Normalize 𝜋i+ = (𝜃+𝛎𝜹i)ᵀx and 𝜋i- = (𝜃-𝛎𝜹i)ᵀx for i = 1 to N
4. Generate 2N episodes and their 2N rewards using 𝜋i+ and 𝜋i- and collect the rewards ri+ and ri-
5. Sort all 𝜹 by max(ri+, ri-)
6. Update 𝜃 = 𝜃 + (𝝰/(b*𝞼ᵣ)) Σ(ri+ - ri-)𝜹i (where i = 1 to b)
7. End While


Policy must have:
Robot state (x,y,velocity)
TG Params (width, height)
Phase of each foot (perhaps?)
Return: TG Params, 8 parameters for leg modifications

    Dimensions: 6 x 10(?)

TG must have:
Input: width, height, timestep (opt)
Find ellipse with width, height, at certain timestep
Return: theta, eps (leg position)


Our policy takes the input: [Roll, Pitch, Yaw, traj_width, traj_height, phase]
The policy output: [4x X, Y residuals, traj_width, traj_height, Phase Offset]


Additional attribution for assistance: https://github.com/OpenQuadruped/spot_mini_mini

ARS Code adapted from bullet: https://github.com/bulletphysics/bullet3/blob/2c204c49e56ed15ec5fcfa71d199ab6d6570b3f5/examples/pybullet/gym/pybullet_envs/ARS/ars.py#L125

'''


import argparse
from multiprocessing import Process, Pipe
import multiprocessing as mp
import time
from gym import wrappers
import gym
import numpy as np
import os
import inspect
import gym_hsa_robot

currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

# Importing the libraries


'''
Need 4 classes here:

Hyperparameters class (opt)
Worker Class
Normalizer
ARS Class
'''

# Setting the Hyper Parameters

class Ellipse_TG():

    def __init__(self):
        self.cycle_length = 240
        self.phase = 0
        self.center_x = 0.0
        self.center_y = -0.074
        self.width = 0.02
        self.height = 0.004
        self.n_params = 2

    def rotate(l, n):
        return l[n:] + l[:n]

    def make_circle(self, x_center, y_center, r_x, r_y, n):

        x_cir = []
        y_cir = []
        for i in range(n):
            xx = r_x * np.sin(2*np.pi * i / n) + x_center
            x_cir.append(xx)

            yy = r_y * np.cos(2*np.pi * i / n) + y_center
            y_cir.append(yy)

        return x_cir, y_cir

    def make_traj(self, offset):
        offset = offset*-1
        test = np.linspace(0, 1, self.cycle_length)
        eps_list = []
        theta_list = []

        for t in test:

            eps = 0.02 * np.cos(2*np.pi*t)
            theta = 0.02 * np.sin(2*np.pi*t)
            eps_list.append(eps)
            theta_list.append(theta)

        eps_list = rotate(eps_list, offset)
        theta_list = rotate(theta_list, offset)

        # print(eps_list, theta_list)

        return eps_list, theta_list

    def joints_to_xy_legframe(self, theta, eps):
        x = (0.07+eps)*np.sin(theta)
        y = -(0.07+eps)*np.cos(theta)
        return x, y

    def xy_legframe_to_joints(self, x, y):
        l = np.linalg.norm([x, y], axis=0)
        # print("l", l.shape)
        theta = np.arctan2(y, x) + 1.5707
        # print("theta", theta.shape)

        eps = l - 0.07

        return theta, eps

    def legframe_to_footframe(self, x, y):
        x_foot = x
        y_foot = y+0.07
        return x_foot, y_foot

    def step_traj(self, width, height, res_x, res_y, step_theta=1, step_time=None):
        '''
        Given: a width, height, find the (theta, eps) that makes sense at the given timestep
        '''
        # OOPS
        self.width = width
        self.height = height
        
        x, y = self.make_circle(0.0, -0.074, width, height, self.cycle_length)
        x = np.asarray(x) + res_x
        y = np.asarray(y) + res_y
        
        
        # for idx, val in enumerate(x):
        #     eps, theta = self.xy_legframe_to_joints(x[idx], y[idx])
            
        theta, eps = self.xy_legframe_to_joints(x, y)

        # print(eps.shape, theta.shape)
        # print("phase:", self.phase)
        
        if step_time:
            self.phase += int( (step_time * self.cycle_length) )
            self.phase = self.phase % (self.cycle_length-1)
            
        # if self.phase >= self.cycle_length:
        #     self.phase = self.phase - self.cycle_length
        # if self.phase < 0:
        #     self.phase = 0
        
        # self.phase = np.clip(self.phase, 0, self.cycle_length-1)
        
        ep_out = eps[int(self.phase)]
        theta_out = theta[int(self.phase)]
        # print(self.phase)
        
        if step_theta or step_time:
            self.phase += step_theta
            self.phase = self.phase % (self.cycle_length-1)

        return ep_out, theta_out


class Hp():

    def __init__(self):
        self.nb_steps = 500
        self.episode_length = 1200
        self.learning_rate = 0.01
        self.nb_directions = 12
        self.nb_best_directions = 4
        assert self.nb_best_directions <= self.nb_directions
        self.noise = 0.002 # previously 0.01
        self.seed = 42
        self.env_name = 'hsa_robot-v0'


class Normalizer():

    def __init__(self, nb_inputs):
        self.n = np.zeros(nb_inputs)
        self.mean = np.zeros(nb_inputs)
        self.mean_diff = np.zeros(nb_inputs)
        self.var = np.zeros(nb_inputs)

    def observe(self, x):
        # print(self.mean, x)
        self.n += 1.
        last_mean = self.mean.copy()
        self.mean += (x - self.mean) / self.n
        self.mean_diff += (x - last_mean) * (x - self.mean)
        self.var = (self.mean_diff / self.n).clip(min=1e-2)

    def normalize(self, inputs):
        obs_mean = self.mean
        obs_std = np.sqrt(self.var)
        return (inputs - obs_mean) / obs_std


class Policy():

    def __init__(self, input_size, output_size, env_name, traj_generator, args=None):

        # self.tg_AC = Ellipse_TG()
        # self.tg_BC = Ellipse_TG()
        # self.tg_BC.phase = 120 # Set the TG's phase to 120, 180 degrees out of phase of the other two legs

        try:
            self.theta = np.load(args.policy)
        except:
            self.theta = np.zeros((output_size, input_size))
        self.env_name = env_name
        self.input_size = input_size
        self.output_size = output_size
        # print("Starting policy theta=", self.theta)
        print("Policy size=", self.theta.shape)

    def evaluate(self, input, delta, direction, hp):
        if direction is None:
            # print(self.theta.shape)
            # print(self.theta)
            # print(input.shape)
            # print(input)
            test = np.dot(self.theta, input)
            # print(test.shape)
            # print(test)
            out_arr = np.clip(test, -1.0, 1.0)
            return  out_arr # Verify that this has the same behavior as the function below
        elif direction == "positive":
            #print((self.theta + hp.noise * delta))
            return np.clip((self.theta + hp.noise * delta).dot(input), -1.0, 1.0)
        else:
            # print(self.theta.shape)
            # print(input.shape)
            #print((self.theta - hp.noise * delta))
            return np.clip((self.theta - hp.noise * delta).dot(input), -1.0, 1.0)

    def sample_deltas(self):
        return [np.random.randn(*self.theta.shape) for _ in range(hp.nb_directions)]

    def update(self, rollouts, sigma_r, args):
        step = np.zeros(self.theta.shape)
        for r_pos, r_neg, d in rollouts:
            step += (r_pos - r_neg) * d
        self.theta += hp.learning_rate / (hp.nb_best_directions * sigma_r) * step
        timestr = time.strftime("%Y%m%d-%H%M%S")

        # np.save(args.logdir + "/policy_" + self.env_name +
        # "_" + timestr + ".npy", self.theta)
        # print(self.theta, self.theta.shape)


def explore(env, normalizer, policy, direction, delta, hp, traj_generators):
    state = env.reset()
    # print("state", state)  # this is an ndarray
    #print(policy.theta)
    done = False
    num_plays = 0.
    sum_rewards = 0
    while not done and num_plays < hp.episode_length: # Formerly: "not done and" as an additional condition

        # print(traj_generators[0].width)
        # tg_params = np.array([traj_generators[0].width, traj_generators[0].height,
        #                       traj_generators[1].width, traj_generators[1].height,
        #                       traj_generators[2].width, traj_generators[2].height,
        #                       traj_generators[3].width, traj_generators[3].height], dtype=float)
        tg_params = np.array([traj_generators[0].width, traj_generators[0].height], dtype=float)
        phase = np.array([traj_generators[0].phase])
        
        state = state[2:]

        state = np.concatenate((state, tg_params, phase), axis=0)
        # print(state)
        # Our state should be 15-dimensional: [x_pos, y_pos, roll, pitch, yaw, x_vel, y_vel, fl_w, fl_h, fr_w, fr_h, rl_w, rl_h, rr_w, rr_h]
        # print("the system's state", state, "length:", len(state))
        # Augment this to include the variables we need from the TG
        normalizer.observe(state)
        
        # THIS DOESN'T PROPERLY APPLY NOISE!!!
        state = normalizer.normalize(state)
        action = policy.evaluate(state, delta, direction, hp)

        # print("action", action)
        # Action is now 16-dimensional: [fl_w, fl_h, res_fl_x, res_fl_y, fr_w, fr_h, res_fr_x, res_fr_y, rl_w, rl_h, res_rl_x, res_rl_y, rr_w, rr_h, res_rr_x, res_rr_y]
        
        # eps_fl, theta_fl = traj_generators[0].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[2]), res_y=0.005*(action[3]))
        # eps_fr, theta_fr = traj_generators[1].step_traj(width=0.015*(1+action[4]), height=0.003*(1+action[5]), res_x=0.023*(action[6]), res_y=0.005*(action[7]))
        # eps_rl, theta_rl = traj_generators[2].step_traj(width=0.015*(1+action[8]), height=0.003*(1+action[9]), res_x=0.023*(action[10]), res_y=0.005*(action[11]))
        # eps_rr, theta_rr = traj_generators[3].step_traj(width=0.015*(1+action[12]), height=0.003*(1+action[13]), res_x=0.023*(action[14]), res_y=0.005*(action[15]))
        #print(action[10])
        # eps_fl, theta_fl = traj_generators[0].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[2]), res_y=0.005*(action[3]), step_time=abs(action[10]))
        # eps_fr, theta_fr = traj_generators[1].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[4]), res_y=0.005*(action[5]), step_time=abs(action[10]))
        # eps_rl, theta_rl = traj_generators[2].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[6]), res_y=0.005*(action[7]), step_time=abs(action[10]))
        # eps_rr, theta_rr = traj_generators[3].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[8]), res_y=0.005*(action[9]), step_time=abs(action[10]))
        
        eps_fl, theta_fl = traj_generators[0].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[2]), res_y=0.005*(action[3]), step_theta=24, step_time=abs(action[10]))
        eps_fr, theta_fr = traj_generators[1].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[4]), res_y=0.005*(action[5]), step_theta=24, step_time=abs(action[10]))
        eps_rl, theta_rl = traj_generators[2].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[6]), res_y=0.005*(action[7]), step_theta=24, step_time=abs(action[10]))
        eps_rr, theta_rr = traj_generators[3].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[8]), res_y=0.005*(action[9]), step_theta=24, step_time=abs(action[10]))

        # Due to PMTG, our action now becomes... 9 + (x_val, y_val, width, height) * 4  = 25 dimensional

        # Change the variable "action" so that it's 9-dimensional (the shape of the environment's input), using the TG
        # Sample from all 4 trajectory generators, make an action from all of them
        # print(aaaaaaaaa)

        # Make sure that the order of legs here is correct
        actions_tg = [0, theta_fl, eps_fl, theta_fr, eps_fr, theta_rl, eps_rl, theta_rr, eps_rr]
        
        # Adding noise to our actions here
        noise = np.random.normal(scale=0.001, size=len(actions_tg))
        actions_tg += noise

        # actions_tg = [0, eps_fl, theta_fl, eps_fr, theta_fr, eps_rl, theta_rl, eps_rr, theta_rr]
        # Here, generate the trajectory from the trajectory generator. Use the actions
        # env.render()
        # print(num_plays)
        for i in range(24):
            state, reward, done, _ = env.step(actions_tg)
            # print(done)
            if done:
                reward = -50
            reward = max(min(reward, 1), -1)
            sum_rewards += reward
            num_plays += 1
        
    print("rollout, cumulative distance in X direction: %f" %sum_rewards)
    return sum_rewards - state[1] # This should ideally prefer walking straight


def train(env, policy, normalizer, hp, traj_generators, args):

    reward_max = 0
    for step in range(hp.nb_steps):


        print(policy.theta)
        deltas = policy.sample_deltas()
        pos_rewards = [0] * hp.nb_directions
        neg_rewards = [0] * hp.nb_directions

        # Getting the positive rewards in the positive directions
        for k in range(hp.nb_directions):
            pos_rewards[k] = explore(
                env, normalizer, policy, "positive", deltas[k], hp, traj_generators)

            # Getting the negative rewards in the negative/opposite directions
        for k in range(hp.nb_directions):
            neg_rewards[k] = explore(
                env, normalizer, policy, "negative", deltas[k], hp, traj_generators)

        all_rewards = np.array(pos_rewards + neg_rewards)
        sigma_r = all_rewards.std()

        # Sorting the rollouts by the max(r_pos, r_neg) and selecting the best directions
        scores = {
            k: max(r_pos, r_neg)
            for k, (r_pos, r_neg) in enumerate(zip(pos_rewards, neg_rewards))
        }
        order = sorted(scores.keys(), key=lambda x: -
                       scores[x])[:hp.nb_best_directions] # sorts and keeps the n best directions
        rollouts = [(pos_rewards[k], neg_rewards[k], deltas[k])
                    for k in order]

        # Updating our policy
        policy.update(rollouts, sigma_r, args)

        # Printing the final reward of the policy after the update
        reward_evaluation = explore(
            env, normalizer, policy, None, None, hp, traj_generators)
        print('Step:', step, 'Reward:', reward_evaluation)
        
        if reward_evaluation > reward_max:
            
            string = "epoch_" + str(step) + "_" + str(reward_evaluation) + ".npy"
            np.save(string, policy.theta)
            
            # Oops, didn't have this before
            reward_max = reward_evaluation
            print("New maximum reward!")


if __name__ == "__main__":
    
    np.set_printoptions(suppress=True)

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--logdir', help='Directory root to log policy files (npy)', type=str, default='.')

    args = parser.parse_args()

    hp = Hp()
    TG_fl = Ellipse_TG()

    TG_fr = Ellipse_TG()
    TG_fr.phase = TG_fr.cycle_length/2

    TG_rl = Ellipse_TG()
    TG_rl.phase = TG_rl.cycle_length/2

    TG_rr = Ellipse_TG()

    tg_arr = [TG_fl, TG_fr, TG_rl, TG_rr]

    print("seed = ", hp.seed)
    np.random.seed(hp.seed)

    # make the environment
    env = gym.make("hsa_robot-v0")
    
    # print("AAA", env.observation_space.shape[0])

    # number of inputs: number of columns
    # number of outputs: number of rows
    # n_inputs = env.observation_space.shape[0] + TG_fl.n_params*4
    n_inputs = env.observation_space.shape[0] + TG_fl.n_params + 1 - 2 #adding the input of phase here -> "-2" signifies removing the XY from the policy
    # n_outputs = env.action_space.shape[0] + 8 + TG_fl.n_params*4
    # THIS DOESN'T EVEN NEED THE ACTION SPACE TO WORK! ONLY NEEDS TRAJ PARAMS
    n_outputs = 8 + TG_fl.n_params + 1 # add 1 here to add an offset to "warp time" per paper - step size added to phase

    print("Observation space =", n_inputs)
    print("Action space =", n_outputs)

    policy = Policy(input_size=n_inputs, output_size=n_outputs,
                    env_name=hp.env_name, traj_generator=tg_arr, args=args)

    normalizer = Normalizer(n_inputs)

    # Now, we start training

    train(env, policy, normalizer, hp, tg_arr, args)

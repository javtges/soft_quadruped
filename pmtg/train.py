import itertools

import numpy as np
import matplotlib.pyplot as plt
import torch

from datetime import datetime
from torch.utils.tensorboard import SummaryWriter

# from src.tg_env import BasicTG
# from src.agent import DDPG
# from src.utils import ReplayMemory


lr = 0.001
tau = 0.001
gamma = 0.99
random_seed = 0
replay_size = 1000000
batch_size = 256
num_steps = 150000
epsilon_start = 1.0
epsilon_end = 0.01
epsilon_decay = 0.98
eval_every_N_episodes = 10
w_decay = 0

state_size = 5
action_size = 4


def plotter(reward_window, env, learned_x, learned_y, i_episode):
    # Plot the data
    plt.close()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    ax1.set_ylabel('Cumulative Reward')
    ax1.set_xlabel('Episode')
    # remove spines
    ax1.spines['right'].set_visible(False)
    ax1.spines['top'].set_visible(False)
    ax1.plot(reward_window)

    # Draw referece fig
    ax2.scatter(env.x, env.y)
    # Draw the agent's path
    ax2.set_ylabel('Y dimension')
    ax2.set_xlabel('X dimension')
    # remove spines
    ax2.spines['right'].set_visible(False)
    ax2.spines['top'].set_visible(False)
    ax2.scatter(learned_x, learned_y)
    # plt.draw()
    # plt.pause(0.001)

    bg_color = '#95A4AD'
    filename = f'images/frame_{i_episode}.png'
    plt.savefig(filename, dpi=96, facecolor=bg_color)


def main():
    env = gym.make('hsa_robot-v0')

    if random_seed is not None:
        torch.manual_seed(random_seed)
        np.random.seed(random_seed)

    agent = DDPG(
            state_size=state_size,
            action_size=action_size,
            lr=lr,
            tau=tau,
            gamma=gamma,
            random_seed=random_seed,
            use_cuda=False,
    )

    memory = ReplayMemory(replay_size)
    tb_writer = SummaryWriter('runs/{}_DDPG_{}'.format(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"), "OptimalPath"))

    eps = epsilon_start

    total_num_steps = 0
    reward_window = []
    
    for i_episode in itertools.count(0):
        episode_reward = 0
        episode_steps = 0
        done = False

        all_x = []
        all_y = []

        state = env.reset()
        agent.reset()

        while not done:
            action = agent.select_action(state, eps, add_noise=True)
            next_state, reward, done, _ = agent.step(env, action)

            memory.push(state, action, reward, next_state, done)

            # Learn, if enough samples are available in memory
            if len(memory) >= batch_size:
                critic_loss, actor_loss = agent.update_parameters(memory, batch_size)

                tb_writer.add_scalar('loss/critic', critic_loss, i_episode)
                tb_writer.add_scalar('loss/actor', actor_loss, i_episode)

            episode_steps += 1
            total_num_steps += 1
            episode_reward += reward
            state = next_state

            all_x.append(next_state[1])
            all_y.append(next_state[2])

            if total_num_steps >= num_steps:
                break

        if total_num_steps >= num_steps:
            break

        reward_window.append(episode_reward)

        eps = max(epsilon_end, epsilon_decay*eps)

        tb_writer.add_scalar('reward/episode', episode_reward, i_episode)

        if i_episode % 10:
            plotter(reward_window, env, all_x, all_y, i_episode)

        print(
            f"Episode: {i_episode}, total_steps: {total_num_steps} \
              episode steps: {episode_steps} \
              return: {round(episode_reward, 2)}"
        )

        if eval_every_N_episodes is not None and (i_episode + 1) % eval_every_N_episodes == 0:
            reward_average = np.mean(reward_window)
            reward_max = np.max(reward_window)
            print('\nEpisode {}\tAverage Score: {:.2f}\tMax: {:.1f}\n'.format(
                i_episode, reward_average, reward_max))
            tb_writer.add_scalar('reward/average', reward_average, i_episode)


if __name__ == '__main__':
    main()
import numpy as np
import random

# from src.models import Actor, Critic
# from src.utils import OUNoise, soft_update

import torch
import torch.nn.functional as F
import torch.optim as optim


class DDPG:
    
    def __init__(
            self,
            state_size,
            action_size,
            lr,
            tau,
            gamma,
            random_seed,
            use_cuda,
    ):

        self.state_size = state_size
        self.action_size = action_size
        self.seed = random.seed(random_seed)
        self.tau = tau
        self.gamma = gamma

        self.device = torch.device("cuda" if torch.cuda.is_available() and use_cuda else "cpu")

        self.actor_local = Actor(self.state_size,
                                 self.action_size,
                                 self.seed).to(self.device)
        self.actor_target = Actor(self.state_size,
                                  self.action_size,
                                  self.seed).to(self.device)
        self.actor_optimizer = optim.Adam(self.actor_local.parameters(),
                                          lr=lr)

        self.critic_local = Critic(self.state_size,
                                   self.action_size,
                                   self.seed).to(self.device)
        self.critic_target = Critic(self.state_size,
                                    self.action_size,
                                    self.seed).to(self.device)
        self.critic_optimizer = optim.Adam(self.critic_local.parameters(),
                                           lr=lr)

        self.noise = OUNoise(self.action_size, self.seed)
    
    
    def reset(self):
        self.noise.reset()

    def step(self, env, action):
        x_a_tg, y_a_tg, nn_modulation_x, nn_modulation_y = action

        next_state = env.step(x_a_tg, y_a_tg, nn_modulation_x, nn_modulation_y)
        _, next_x, next_y, _, _ = next_state

        reward = env.reward(next_x, next_y)
        done = env.is_done()
        info = None

        return next_state, reward, done, info

    def select_action(self, state, eps, add_noise=True):
        state = torch.from_numpy(np.asarray(state)).float().to(self.device)
        self.actor_local.eval()
        with torch.no_grad():
            action = self.actor_local(state).cpu().data.numpy()
        self.actor_local.train()
        if add_noise:
            action += eps * self.noise.sample()
        return np.clip(action, -1, 1)

    def update_parameters(self, memory, batch_size):
        states, actions, rewards, next_states, dones = memory.sample(batch_size=batch_size)

        states = torch.from_numpy(states).float().to(self.device)
        actions = torch.from_numpy(actions).float().to(self.device)
        rewards = torch.from_numpy(rewards).float().to(self.device)
        next_states = torch.from_numpy(next_states).float().to(self.device)
        dones = torch.from_numpy(dones).float().to(self.device)

        # -------------------------- update critic -------------------------- #
        # Get predicted next-state actions and Q values from target models
        next_actions = self.actor_target(next_states)
        next_q_targets = self.critic_target(next_states, next_actions)
        # Compute Q targets for current states (y_i)
        q_targets = rewards + (self.gamma * next_q_targets * (1 - dones))
        # Compute critic loss
        q_expected = self.critic_local(states, actions)
        critic_loss = F.mse_loss(q_expected, q_targets)
        # Minimize the loss
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # -------------------------- update actor -------------------------- #
        # Compute actor loss
        actions_pred = self.actor_local(states)
        actor_loss = -self.critic_local(states, actions_pred).mean()
        # Minimize the loss
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # --------------------- update target networks --------------------- #
        soft_update(self.critic_local, self.critic_target, self.tau)
        soft_update(self.actor_local, self.actor_target, self.tau)

        return {
                critic_loss: critic_loss.item(),
                actor_loss: actor_loss.item()
                }
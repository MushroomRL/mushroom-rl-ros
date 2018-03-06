#!/usr/bin/env python

import numpy as np

from mushroom_ros.environments import TurtlebotGazebo

from mushroom.core import Core
from mushroom.algorithms.policy_search import REINFORCE
from mushroom.approximators.parametric import LinearApproximator
from mushroom.approximators.regressor import Regressor
from mushroom.features.features import Features
from mushroom.features.tensors import gaussian_tensor
from mushroom.policy import GaussianPolicy, MultivariateGaussianPolicy, MultivariateDiagonalGaussianPolicy
from mushroom.utils.dataset import compute_J
from mushroom.utils.parameters import Parameter, AdaptiveParameter


# Learning parameters
n_runs = 4
n_iterations = 10
ep_per_run = 3

# Environment
mdp = TurtlebotGazebo()

# Policy
tensor_list = gaussian_tensor.generate([10, 10, 6],
                                       [[-5.0, 5.0],
                                        [-5.0, 5.0],
                                        [-np.pi, np.pi]])

phi = Features(tensor_list=tensor_list, name='phi',
               input_dim=mdp.info.observation_space.shape[0])


input_shape = (phi.size,)

approximator_params = dict(input_dim=phi.size)
approximator = Regressor(LinearApproximator, input_shape=input_shape,
                         output_shape=mdp.info.action_space.shape,
                         params=approximator_params)

sigma = np.eye(2)*1e-1
policy = MultivariateGaussianPolicy(mu=approximator, sigma=sigma)


# Agent
learning_rate = AdaptiveParameter(value=5)
algorithm_params = dict(learning_rate=learning_rate)
fit_params = dict()
agent_params = {'algorithm_params': algorithm_params,
                'fit_params': fit_params}
agent = REINFORCE(policy, mdp.info, agent_params, phi)

# Train
core = Core(agent, mdp)
print 'Initial evaluation'
dataset_eval = core.evaluate(n_episodes=ep_per_run)
J = compute_J(dataset_eval, gamma=mdp.info.gamma)
print('J at start : ' + str(np.mean(J)))

for i in xrange(n_runs):
    print 'iteration', i
    print 'learn'
    core.learn(n_episodes=n_iterations * ep_per_run,
               n_episodes_per_fit=ep_per_run)
    print 'evaluate'
    dataset_eval = core.evaluate(n_episodes=ep_per_run)
    J = compute_J(dataset_eval, gamma=mdp.info.gamma)
    print('J at iteration ' + str(i) + ': ' + str(np.mean(J)))
    
mdp.stop()


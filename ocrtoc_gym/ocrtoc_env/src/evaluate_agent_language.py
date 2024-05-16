import datetime
import os
import numpy as np
from joblib import Parallel, delayed
import gymnasium as gym
from ocrtoc_agent.agent_builder import MyAgent
from time import time
from ocrtoc_env.src.env_generator_language import sense_generate
def evaluate(object_category,object_max_num,overlap,object_class, n_episodes=1080, n_cores=-1, render=False,  **kwargs):
    """
    Function that will run the evaluation of the agent for a given set of environments.

    Args:
        n_episodes (int, 1080): Number of episodes each environment is evaluated
        n_cores (int, -1): Number of parallel cores which are used for the computation. -1 Uses all cores.
            When using 1 core the program will not be parallelized (good for debugging)
        render (bool, False): set to True to spawn a viewer that renders the simulation
        kwargs (any): Argument passed to the Agent init
    """
    ## create random env
    sense_generate(object_category,object_max_num,overlap,object_class)

    # set system parameter
    if n_cores == -1:
        n_cores = os.cpu_count()

    assert n_cores != 0
    assert n_episodes >= n_cores

    env_init_chuncks = []

    # Precompute all initial states for all experiments
    # for env in env_list:
    env_init_chuncks.append(generate_init_states(n_episodes, n_cores))

    ## Log TODO
    # log_file_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'result')
    # if not os.path.exists(log_file_dir):
    #     os.makedirs(log_file_dir)
    # f = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'result', datetime.datetime.now().strftime('eval-%Y-%m-%d_%H-%M-%S.txt')), "a")
    for chunks in env_init_chuncks:
        # evaluate 
        data= Parallel(n_jobs=n_cores)(delayed(_evaluate)(chunks[i],  render,  **kwargs) for i in range(n_cores))
        # write score into files
        # average_score = 0  ##TODO
        # average_score_time = 0 ## TODO

        # print("OCROTC task: LANGUAGE average score in ", n_episodes, "episodes is ", average_score)
    #     f.write("OCROTC task: LANGUAGE average score in " + str(n_episodes) + " episodes is " + str(average_score) + " Mean Rearrengement per hour is " + str(average_score_time))
    #     f.write("\n")
    # f.close()
   


def _evaluate(init_states, render,**kwargs):
    """
        evaluate one episode 
    """
    sum_scene_score = 0
    sum_scene_score_time = 0 
    if render == True:
        gym_env = gym.make("OCRTOC_Language-v0",render_mode="human")
    else:
        gym_env = gym.make("OCRTOC_Language-v0",render_mode="rgb_array")
    target_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname((os.path.abspath(__file__)))))),"ocrtoc_materials_mujoco","targets","language_conditioned_target.yaml")
    for k in range(len(init_states)):
        my_agent = MyAgent(target_path, **kwargs)
        observation, info = gym_env.reset()
        start_time = time()

        for _ in range(6000):
            action, success= my_agent.draw_action(observation)
            observation, _ , _, _ ,_ = gym_env.step(action)
            if success:
                break
         
        end_time = time()
        seconds_elapsed = end_time - start_time
        # score_calculator = ScoreCalculator(env = gym_env, task_index = env, IoU = IoU_threshold, time_cost = seconds_elapsed)
        # scene_score, scene_score_time = score_calculator.calculate_score(debug)
        sum_scene_score_time = 0#sum_scene_score_time + scene_score_time
        sum_scene_score = 0#sum_scene_score + scene_score
    gym_env.close()

    return sum_scene_score , sum_scene_score_time 


def generate_init_states(n_episodes, n_parallel_cores):
    gym_env = gym.make("OCRTOC_Language-v0",render_mode="rgb_array")
    init_states = []
    chunk_lens = [n_episodes // n_parallel_cores + int(x < n_episodes % n_parallel_cores) for x in
                  range(n_parallel_cores)]
    for chunk_len in chunk_lens:
        init_states_chunk = np.zeros((chunk_len,gym_env.unwrapped.model.key_qpos.size))
        for i in range(chunk_len):
            init_states_chunk[i] = gym_env.unwrapped.model.key_qpos.copy()
        init_states.append(init_states_chunk)

    return init_states

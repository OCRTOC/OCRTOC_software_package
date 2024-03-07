import sys
import os
sys.path.append(os.path.dirname((os.path.abspath(__file__))))
import ocrtoc_env
# from ocrtoc_agent.agent_builder import DummyAgent
from argparse import ArgumentParser
from pathlib import Path
import yaml
from ocrtoc_env.src.evaluate_agent import evaluate

# convert shortcut to full list
def convert_envs(env_list):
    if "1_1" in env_list:
        env_list.remove("1_1")
        env_list.extend(["1_1_1","1_1_2","1_1_3"])
    if "1_2" in env_list:
        env_list.remove("1_2")
        env_list.extend(["1_2_1","1_2_2"])
    if "1_3" in env_list:
        env_list.remove("1_3")
        env_list.extend(["1_3_1"])
    if "1_4" in env_list:
        env_list.remove("1_4")
        env_list.extend(["1_4_1","1_4_2"])
    if "1_5" in env_list:
        env_list.remove("1_5")
        env_list.extend(["1_5_1","1_5_2","1_5_3"]) 
    if "2_1" in env_list:
        env_list.remove("2_1")
        env_list.extend(["2_1_1","2_1_2"])     
    if "2_2" in env_list:
        env_list.remove("2_2")
        env_list.extend(["2_2_1","2_2_2","2_2_3","2_2_4"]) 
    if "3_1" in env_list:
        env_list.remove("3_1")
        env_list.extend(["3_1_1","3_1_2"])   
    if "3_2" in env_list:
        env_list.remove("3_2")
        env_list.extend(["3_2_1","3_2_2"])
    if "3_3" in env_list:
        env_list.remove("3_3")
        env_list.extend(["3_3_1","3_3_2"])
    if "4_1" in env_list:
        env_list.remove("4_1")
        env_list.extend(["4_1_1","4_1_2"])   
    if "4_2" in env_list:
        env_list.remove("4_2")
        env_list.extend(["4_2_1","4_2_2","4_2_3","4_2_4"])
    if "4_3" in env_list:
        env_list.remove("4_3")
        env_list.extend(["4_3_1","4_3_2","4_3_3","4_3_4"])
    if "5_2" in env_list:
        env_list.remove("5_2")
        env_list.extend(["5_2_1","5_2_2"])     
    if "6_1" in env_list:
        env_list.remove("6_1")
        env_list.extend(["6_1_1","6_1_2"])  
    if "6_2" in env_list:
        env_list.remove("6_2")
        env_list.extend(["6_2_1","6_2_2","6_2_3"]) 
    if "6_3" in env_list:
        env_list.remove("6_3")
        env_list.extend(["6_3_1","6_3_2","6_3_3"])
    return env_list

# load argument from terminal
def get_args():
    parser = ArgumentParser()
    arg_test = parser.add_argument_group('override parameters')

    arg_test.add_argument("-e", "--env", nargs='*',
                          help='Environments to be used.')

    arg_test.add_argument("--n_cores", type=int, help="Number of CPU cores used for evaluation.")

    arg_test.add_argument("-n", "--n_episodes", type=int,
                          help="Each seed will run for this number of Episodes.")

    default_path = Path(__file__).parent.joinpath("ocrtoc_agent/agent_config.yml")
    arg_test.add_argument("-c", "--config", type=str, default=default_path,
                          help="Path to the config file.")

    arg_test.add_argument("-r", "--render", help="If set renders the environment")

    args = vars(parser.parse_args())
    return args


if __name__ == "__main__":
    args = get_args()
    # Remove all None entries
    filtered_args = {k: v for k, v in args.items() if v is not None}
    
    # Load config
    if os.path.exists(filtered_args["config"]):
        with open(filtered_args["config"]) as stream:
            config = yaml.safe_load(stream)
    else:
        print("Could not read config file with path: ", filtered_args["config"])
        # config = {"render": False}
    del filtered_args["config"]

    config.update(filtered_args)
    config["env_list"] = convert_envs(config["env"])
    del config["env"]
    print("config:", config)

    evaluate(**config)

import os
from gymnasium.envs.registration import register


ENV_IDS = []

## load pose conditioned tasks
src = "../ocrtoc_materials_mujoco/scenes/"
for item in os.listdir(src):
        s = os.path.join(src, item)
        if s.endswith(".xml"):
            _, rest = s.rsplit('scenes/', 1)
            index, extension = rest.rsplit('.', 1)
            index_name = index.replace("-","_")
            task = index_name
            env_id = f"OCRTOC_{task}-v0"

            register(
                id=env_id,
                entry_point=f"ocrtoc_env.src.load_all_task:OCRTOC_task_{task}",
                max_episode_steps=50,
            )

            ENV_IDS.append(env_id)

## load language conditioned tasks
env_id = f"OCRTOC_Language-v0"

register(
    id=env_id,
    entry_point=f"ocrtoc_env.src.load_all_task:OCRTOC_task_language",
    max_episode_steps=50,
)

ENV_IDS.append(env_id)
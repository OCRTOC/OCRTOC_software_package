#! /root/ocrtoc_ws/src/conda_env/bin/python
# Kill all ocrtoc related processes
# we does not assume any high level package installed
#
# Note: please update your conda_env or run `pip install psutil`
#
# By Jet <c@jetd.me>, MAR 2021
#
import time
import psutil

OCRTOC_KEYWORDS = [
    'commit_solution', 'sapien_env', 'trigger_and_evaluation'
    'move_group', 'camera_info_pub',
    'rosout', 'rosmaster', 'ros_launch',
    'sim_depth_node', 'robot_state_pub'
]


def is_ocrtoc_process(name):
    for k in OCRTOC_KEYWORDS:
        if k in name:
            return True
    return False


def is_ocrtoc_clean():
    for proc in psutil.process_iter(['name', 'pid', 'status']):
        if is_ocrtoc_process(proc.info['name']) \
           and proc.info['status'] not in [psutil.STATUS_ZOMBIE, psutil.STATUS_DEAD]:
            return False, proc.info
    return True, None


def kill_ocrtoc(nchecks=1, interval=1.0):
    for _ in range(nchecks):
        for proc in psutil.process_iter(['pid', 'name']):
            if is_ocrtoc_process(proc.info['name']):
                psutil.Process(proc.info['pid']).kill()
        time.sleep(interval)


if __name__ == '__main__':
    print(f'ocrtoc clean: {is_ocrtoc_clean()}')
    print('Trying to kill ocrtoc processes...')
    kill_ocrtoc(nchecks=3, interval=3.0)
    print(f'ocrtoc clean: {is_ocrtoc_clean()}')


import os
import pybullet
import os.path as path

# connect to simulation
physicsClientId = pybullet.connect(pybullet.SHARED_MEMORY)

folder = path.abspath(path.join(__file__, "../../..")) + '/scenes/sim_dev_scenes'
index = '1-1'
world = 'input.world'
world_path = folder + '/' + index + '/' + world

pybullet.loadSDF(world_path)



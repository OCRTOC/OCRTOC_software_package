import os
import random
import math
import numpy as np
import xml.etree.ElementTree as ET
from xml.etree import ElementTree
from xml.dom import minidom
from scipy.spatial.transform import Rotation as Rotation
import yaml
def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

# trans list to string
def list_to_string(data):
    string = ""
    for i in data:
        string = string + " " + str(i)  
    return string


## calculate min distance between new object and exited object
def calculate_min_distance(pos,pos_list):
    min_distance = 100
    for i in range(len(pos_list)):
        dist = math.sqrt( (pos[0] - pos_list[i][0])**2 + (pos[1] - pos_list[i][1])**2 )
        if min_distance > dist:
            min_distance = dist
    return min_distance

## generate grasp sense
def sense_generate(objects,objects_num,overlap,object_class):
    # random generate env
    if object_class == "tool":
        object_names= ["flat_screwdriver", "mini_claw_hammer_1",   "two_color_hammer", "phillips_screwdriver","large_clamp","extra_large_clamp"] # "power_drill,,, "
    elif object_class == "grocery":
        object_names= ["blue_cup","green_bowl","orion_pie","suger_2","suger_3"] # "power_drill,,, "
    elif object_class == "fruit":
        object_names= ["plastic_apple","plastic_banana","plastic_lemon","plastic_peach","plastic_pear","plastic_strawberry"] # "power_drill,,, "
    else: 
        raise AssertionError("Wrong object class")
    object_selected = random.sample(object_names, objects) # random select n objects
    print("picked",object_selected)
    objects_in_sense = []   # objects in mujoco 
    objectnum_list = []  # number of each object
    boxes_name = []
    for item in object_selected:
        objectnum_list.append(random.randrange(1,objects_num+1))
        boxes_name.append(item+"_box__1")
        for i in range(objectnum_list[-1]):
            objects_in_sense.append(item+"__"+str(i))

    ocrtoc_home_path =os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))
    src_model = ocrtoc_home_path + "/ocrtoc_materials_mujoco/models/"
    box_name = "clear_box_1"
    box_offset = 0.35  #regarding box size 
    # create new tree 
    output_tree = ET.Element('mujoco',{'model':'Language_conditioned_Task_Env'})
    tree_include_basic = ET.SubElement(output_tree, 'include',{'file':'../basic_scene_language.xml'})
    tree_include_panda_asset = ET.SubElement(output_tree, 'include',{'file':'../panda/panda_asset.xml'})
    tree_include_panda_actuator = ET.SubElement(output_tree, 'include',{'file':'../panda/panda_actuator.xml'})

  
    for item in object_selected:
        tree_include =  ET.SubElement(output_tree, 'include',{'file':'../models/'+ item + '/' + item + '_asset.xml'})
    tree_include =  ET.SubElement(output_tree, 'include',{'file':'../models/'+ box_name + '/' + box_name + '_asset.xml'})

    tree_compile = ET.SubElement(output_tree, 'compiler',{'angle':'radian', 'autolimits':'true'})
    # tree_compile = ET.SubElement(output_tree, 'option',{'timestep':'0.001'})

    tree_include_worldbody = ET.SubElement(output_tree, 'worldbody')
    tree_include_worldbody_body =  ET.SubElement(tree_include_worldbody, 'body',{'name':'panda', 'pos':'-0.42 0.0 0.0', 'euler':'0 0 0'})
    tree_include_worldbody_body_include = ET.SubElement(tree_include_worldbody_body, 'include',{'file':'../panda/panda_chain.xml'})
   

    # load object1
    pos_list = []
    init_pos = []
    R = 0.3 #object inital area
    init_height = 0.05 # object inital height
    for item in object_selected:
        object_num = objectnum_list[object_selected.index(item)]
        for i in range(object_num):
                phi = np.random.random(1) * np.pi
                r = np.sqrt(np.random.random(1)) * R
                y =- r * np.cos(phi)
                x = r * np.sin(phi) *1.1
                if not overlap:
                    pos = [((x[0]-0.2)+0.1)+0.03 ,y[0]*1.2,init_height]
                    if i !=0 or object_selected.index(item)!=0:
                        while calculate_min_distance(pos,pos_list)<0.19:
                            phi = np.random.random(1) * np.pi
                            r = np.sqrt(np.random.random(1)) * R
                            y =- r * np.cos(phi) 
                            x = r * np.sin(phi) *1.1
                            pos = [((x[0]-0.2)+0.1)+0.03,y[0]*1.2,init_height]            
                    pos_list.append(pos)
                else:
                    pos = [((x[0]-0.2)+0.1)+0.03 ,y[0]*1.2,init_height]

                euler = [0,0,random.uniform(-3.14, 3.14)]
                # remove some poses which are hard to be grasped
                if pos[1] >=0.1:
                    euler[2] = random.uniform(-1.5, 1.5)
                tree_include_worldbody_body =  ET.SubElement(tree_include_worldbody, 'body',{'name':item+'__'+str(i), 'pos':list_to_string(pos), 'euler':list_to_string(euler)})
                rotation = Rotation.from_euler('XYZ', euler, degrees=False)
                q = rotation.as_quat()
                init_pos.append(pos[0])
                init_pos.append(pos[1])
                init_pos.append(pos[2])
                init_pos.append(q[3])
                init_pos.append(q[0])
                init_pos.append(q[1])
                init_pos.append(q[2])

                # tree_include_worldbody_body_inertial =  ET.SubElement(tree_include_worldbody_body, 'freejoint')
                tree_include_worldbody_body_inertial =  ET.SubElement(tree_include_worldbody_body, 'joint',{'type':'free','damping':'1.0','armature':'1.0'})
                # tree_include_worldbody_body_inertial =  ET.SubElement(tree_include_worldbody_body, 'inertial',{'pos':'0 0 0', 'mass':'0.1', 'fullinertia':'1 1 1 0 0 0'})
                model_folder = src_model + item
                for file in os.listdir(model_folder):
                    if file.startswith('collision_decomposition'):
                        name_, extension = file.rsplit('.', 1)
                        _, decomposition_index = name_.rsplit('decomposition', 1)
                        tree_assert_mesh_coll = ET.SubElement(tree_include_worldbody_body, 'geom',{'name':item +'_'+str(i)+ '_col' + decomposition_index, 'type':'mesh', 'group':'3', 'friction':"1.0 0.005 0.0001", 'pos':"0 0 0", 'euler':"0 0 0", 'mesh':item + '_collision' + decomposition_index , 'solimp':".99 .99 .01", 'solref':".001 1", 'density':'1000'})
                # tree_include_worldbody_body_collision =  ET.SubElement(tree_include_worldbody_body, 'geom',{'name':model_name + '_col', 'class':name, 'type':'mesh', 'group':'3', 'friction':"0.5 0.005 0.0001", 'pos':"0 0 0", 'euler':"0 0 0", 'mesh':name + '_collision'})
                tree_include_worldbody_body_texture =  ET.SubElement(tree_include_worldbody_body, 'geom',{'name':item + '_tex'+'_'+str(i),  'type':'mesh', 'group':'1', 'pos':"0 0 0", 'euler':"0 0 0", 'mesh':item + '_textured', 'material':item ,'contype':'0', 'conaffinity':'0'})
                if overlap:
                    init_height += 0.1
    # create boxes according to the number of objects
    pos =  [-0.9+0.03, 0.4, 0.15] #box pose  
    euler = [0,0,0] #box euler
    k = 1
    for item in object_selected:
        tree_include_worldbody_body =  ET.SubElement(tree_include_worldbody, 'body',{'name':item+'_box__1', 'pos':list_to_string(pos), 'euler':list_to_string(euler)})
        rotation = Rotation.from_euler('XYZ', euler, degrees=False)
        q = rotation.as_quat()
        init_pos.append(pos[0])
        init_pos.append(pos[1])
        init_pos.append(pos[2])
        init_pos.append(q[3])
        init_pos.append(q[0])
        init_pos.append(q[1])
        init_pos.append(q[2])
        if k < 2:
            pos[0] = pos[0] + box_offset #pos of next box
        elif k < 3:
            pos[1] = -pos[1]
        elif k<4:
            pos[0] = pos[0] - box_offset
        k+=1
        # tree_include_worldbody_body_inertial =  ET.SubElement(tree_include_worldbody_body, 'freejoint')
        tree_include_worldbody_body_inertial =  ET.SubElement(tree_include_worldbody_body, 'joint',{'type':'free','damping':'1.0','armature':'1.0'})
        # tree_include_worldbody_body_inertial =  ET.SubElement(tree_include_worldbody_body, 'inertial',{'pos':'0 0 0', 'mass':'0.1', 'fullinertia':'1 1 1 0 0 0'})
        model_folder = src_model + box_name
        for file in os.listdir(model_folder):
            if file.startswith('collision_decomposition'):
                name_, extension = file.rsplit('.', 1)
                _, decomposition_index = name_.rsplit('decomposition', 1)
                tree_assert_mesh_coll = ET.SubElement(tree_include_worldbody_body, 'geom',{'name':item+'_box_'+str(i)+ '_col' + decomposition_index, 'type':'mesh', 'group':'3', 'friction':"1.0 0.005 0.0001", 'pos':"0 0 0", 'euler':"0 0 0", 'mesh':box_name + '_collision' + decomposition_index , 'solimp':".99 .99 .01", 'solref':".001 1", 'density':'50000'})
        # tree_include_worldbody_body_collision =  ET.SubElement(tree_include_worldbody_body, 'geom',{'name':model_name + '_col', 'class':name, 'type':'mesh', 'group':'3', 'friction':"0.5 0.005 0.0001", 'pos':"0 0 0", 'euler':"0 0 0", 'mesh':name + '_collision'})
        tree_include_worldbody_body_texture =  ET.SubElement(tree_include_worldbody_body, 'geom',{'name':item+'_box_'+ '_tex'+'_'+str(i),  'type':'mesh', 'group':'1', 'pos':"0 0 0", 'euler':"0 0 0", 'mesh':box_name + '_textured', 'material':box_name ,'contype':'0', 'conaffinity':'0'})
    
    # create home key
    tree_include_keyframe = ET.SubElement(output_tree, 'keyframe')
    key_qpos = '0.0 0.0 0.0 -1.57 0.0 1.57 0.78' + ' 0 0'+ list_to_string(init_pos)
    tree_include_keyframe_key = ET.SubElement(tree_include_keyframe, 'key',{'name':'home', 'qpos':key_qpos})
    # print(prettify(output_tree))
    save_path = ocrtoc_home_path + "/ocrtoc_materials_mujoco/scenes" + '/language_conditioned_env' +'.xml'
    et = ET.ElementTree(output_tree)
    ET.indent(et, space="\t", level=0)
    with open(save_path, 'wb') as f:
        et.write(f,  xml_declaration=True)
    ## write target prompt
    write_description(ocrtoc_home_path + "/ocrtoc_materials_mujoco/targets/language_conditioned_target.yaml",object_selected)

## write prompts to yaml 
def write_description(list_path, grasp_object_names):
    annotations = []
    for i in range(len(grasp_object_names)):
        annotations.append("grasp " + grasp_object_names[i] + " into box " +str(i+1))
    dct = {"Prompts":annotations}
    
    with open(list_path, 'w') as out_file:
        yaml.dump(dct, out_file,default_flow_style=False, sort_keys=False)   

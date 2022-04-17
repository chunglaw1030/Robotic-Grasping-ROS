#!/usr/bin/python2

"""
This code contains the world spawner. This file accomplishes two main goals:

  1. Create and manage the objects in the gazebo world using the World() class
  2. Spawn and monitor the completion of coursework tasks using the Task() class

The coursework contains three tasks, and each of them are defined here. There
are three classes derived from the Task() base class, Task1(), Task2(), and
Task3().

Each of these TaskX() classes defines what objects will spawn for a given task,
as well as sending the service request for taskX_start() - this is the request
that you will receive and respond to (by solving the task).

Currently, tasks may spawn with random box positions and colours. If you wish to
change this, you can edit the task parameters. First, there are a set of global
parameters (IN UPPER CASE) which you can change. Second, look at the class
methods in each TaskX():
  - spawn_trial_course : this sets up the task and places the models
  - spawn_trial_validation : this begins the task

You will notice that each TaskX() class has another set of empty methods:
  - start_test_course
  - start_test_validation
We have an identical copy of this file, but with those methods fully defined.
When we mark your work, we will run this file with those methods. You do not
need to use those methods.

Best of luck!
"""


import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from math import cos, sin
import numpy as np

# # hint: an easy way to 'disable' the randomness in the task spawning is:
# myseed = 0
# np.random.seed(myseed) # choose any int as your seed

from cw3_world_spawner_lib.coursework_world_spawner import *
from cw3_world_spawner.srv import TaskSetup, TaskSetupResponse
from cw3_world_spawner.srv import Task1Service, Task2Service, Task3Service
from cw3_world_spawner.srv import Task2ServiceRequest
from cw3_world_spawner.srv import Task3ServiceRequest

# ----- user defined coursework parameters ----- #

# TESTING PARAMETERS YOU CAN CHANGE
T1_N_BOXES = 5
T2_N_BOXES = 7
T2_STACK_SIZE = 3
T3_N_BOXES = 10
T3_STACK_SIZE = 4
T1_TEST_TIME = 120     # Testing time for T1 in sec, before timeout
T2_TEST_TIME = 120     # Testing time for T2 in sec, before timeout
T3_TEST_TIME = 360     # Testing time for T3 in sec, before timeout
BOX_COLORS = {'purple': [0.8, 0.1, 0.8],       # Colours and their RGB values
              'red':    [0.8, 0.1, 0.1], 
              'blue':   [0.1, 0.1, 0.8],
              'yellow': [1.0, 1.0, 0.0],
              'orange': [0.9, 0.4, 0.1],
              'pink':   [0.9, 0.7, 0.7]}
BASKET_LOCATIONS = [ (0.6,-0.33), (0.6,0.33) ] # Locations for basket/goal stack


# WORLD SPAWN PARAMETERS - Will be the same for grading. Do not touch.
T1_BOX_X_LIMS = [0.3, 0.5]      #  Limits for box spawns for Task1
T1_BOX_Y_LIMS = [-0.12, 0.12]
T2_BOX_X_LIMS = [0.35, 0.45]     #  Limits for box spawns for Task2
T2_BOX_Y_LIMS = [-0.4, 0.5]
T3_BOX_X_LIMS = [-0.45 ,0.45]   #  Limits for box spawns for Task3
T3_BOX_Y_LIMS = [-0.4,0.5]
TILE_SIDE = 0.1       # Side length of ground tile in m, from sdf
BASKET_SIDE = 0.2     # Side length of basket side in m, from sdf
TILE_HEIGHT = 0.3     # Side height of ground tile in m, from sdf
TILE_Z = 0.01         # Spawn Z value for golf tiles
GOAL_Z_OFFSET = 0.2   # Offset of goal location compared to goal tile.
ROBOT_SAFETY_RADIUS_TILE = 0.25 #  Radius around robot that shouldn't have tiles
ROBOT_SAFETY_RADIUS_BOX = 0.28 #  Radius around robot that shouldn't have objects
TILE_X_LIMS = [-0.45,0.7]  #  Limits for green tiles
TILE_Y_LIMS = [-0.4 ,0.5]
BOX_SIDE = 0.04

def L2_dist(a,b):
  s = 0
  for i in range(len(a)):
    s += (a[i]-b[i])**2
  return s**0.5

# ----- Class definitions for base world and task ----- #

class World(object):
  def __init__(self):
    world_spawner.despawn_all(exceptions="object-golf-tile")
    self.spawn_tiles()


  def spawn_tiles (self):
    ys = np.arange(TILE_Y_LIMS[0], TILE_Y_LIMS[1], step=TILE_SIDE)
    xs = np.arange(TILE_X_LIMS[0], TILE_X_LIMS[1], step=TILE_SIDE)
    for i, x_i in enumerate(xs):
      for j, y_j in enumerate(ys):
        if np.abs(x_i) < ROBOT_SAFETY_RADIUS_TILE and np.abs(y_j)< ROBOT_SAFETY_RADIUS_TILE : 
          continue
        model = Model( model_name = "object-golf-tile",
                      instance_name = 'object-golf-tile_%02d_%02d'%(i,j),
                      model_type = 'sdf',
                      position = [x_i, y_j, TILE_Z ]
                    )
        success = world_spawner.spawn(model)
    return


  def randomize_goal(self, goal_locs = BASKET_LOCATIONS):
    self.goal_locs = goal_locs
    world_spawner.despawn_all('object-goal-tile')
    world_spawner.despawn_all('object-goal-basket')
    rand_id = np.random.randint(0,len(self.goal_locs))
    goal_loc = self.goal_locs[rand_id]
    self.goal_pt = Point(goal_loc[0], goal_loc[1], GOAL_Z_OFFSET)
    model = Model( model_name = "basket",
                  instance_name = "object-goal-basket",
                  model_type = 'sdf',
                  position = [goal_loc[0], goal_loc[1], GOAL_Z_OFFSET]
                )
    success = world_spawner.spawn(model)
      
    return self.goal_pt


class Task(object):
  def __init__(self, mode='coursework', validation_scenario=0):
    self.models = {}
    self.reset_world()
    if mode == 'coursework':
      self.start_trial_setup()
      self.start_trial_validation()
    else:
      self.start_test_setup(validation_scenario)
      self.start_test_validation(validation_scenario)
    return 


  def spawn_box_object(self, name='boxobject1', 
                       xlims = [0.3,0.5], ylims = [-0.12,0.12], zlims = [0.1,0.101],
                       color='blue', z_rotation=None):
    # Spawn ball at random location
    if color=='blue': model_name = 'box_blue'
    elif color=='red': model_name = 'box_red'
    elif color=='purple': model_name = 'box_purple'
    elif color=='yellow': model_name = 'box_yellow'
    elif color=='orange': model_name = 'box_orange'
    elif color=='pink': model_name = 'box_pink'
    else: model_name = 'box'
    rospy.logdebug("Spawning " + model_name)

    if z_rotation is not None:
      q = [0, 0, 0, 0]
      q[0] = cos(z_rotation / 2)
      q[1] = sin(z_rotation / 2)
    else:
      q = [0, 0, 0, 1]

    model = Model(model_name = model_name,
                  instance_name = name,
                  model_type = 'sdf',
                  position = random_position_in_area(xlims, ylims, zlims),
                  orientation = q)
    success = world_spawner.spawn(model)
    return model


  def get_position_from_point(self,pt):
    return np.asarray([pt.x, pt.y, pt.z])

  def get_position_from_point_stamped(self,ptst):
    pt = ptst.point
    return np.asarray([pt.x, pt.y, pt.z])


  def get_position_from_pose(self,pose):
    pos_np = self.get_position_from_point(pose.position)
    return pos_np


  def get_euclidean_distance(self,a,b):
    return np.sqrt(np.sum(np.power(a - b,2)))


  def reset_world(self): 
    world_spawner.despawn_all(keyword='object',exceptions='golf')
    return
  def spawn_trial_course(): pass
  def start_trial_setup(self):
    self.spawn_trial_course()
  def start_trial_validation(self): pass
  def start_test_setup(self, validation_scenario): pass
  def start_test_validation(self, validation_scenario): pass
  
# ----- Class definitions for the coursework tasks ----- #

class Task1(Task):
  def __init__(self, mode='coursework', validation_scenario=None):
    rospy.loginfo('================Starting Task1==============')
    Task.__init__(self, mode, validation_scenario)


  def spawn_trial_course(self):
    """ Spawns trial course - feel free to edit """
    world_spawner.despawn_all(keyword='object',exceptions='golf')
    
    chosen_colors = self.spawn_column(n_obj=T1_N_BOXES,
                          column_name='object_stack',
                          xlims = T1_BOX_X_LIMS,
                          ylims = T1_BOX_Y_LIMS)

    self.goal_pos, self.goal_ori, self.goal_column = chosen_colors
    return


  def spawn_column(self, n_obj, xlims, ylims, column_name='object_stack', colors=None):
    """
    Spawn a pile of cubes one on top of another
    column_name should have 'object' in it if you want the object to be despawned
    upon task completion.
    """

    chosen_colors = []
    z = 45e-3
    z_start = 45e-3

    pos = random_position_in_area(xlims, ylims, [z,z])
    rotation = (np.pi / 180.0) * np.random.randint(0, 90)

    for i in range(n_obj):
      # z spawn position for this cube
      name = '%s_%d' % (column_name, i)

      if colors is None:
        all_colors = list(BOX_COLORS.keys())
        color = all_colors[np.random.choice(len(all_colors))]
      else:
        color = colors[i]

      # create the model instance
      self.models[name] = self.spawn_box_object(name=name,
                                                xlims = [pos[0], pos[0]],
                                                ylims = [pos[1], pos[1]],
                                                zlims = [pos[2]+z*i + z_start, pos[2]+z*i + z_start],
                                                color = color,
                                                z_rotation = rotation)
      chosen_colors.append(color)
       
    return pos, rotation, chosen_colors


  def reset_world(self): pass


  def prepare_for_task_request(self):
    rospy.logdebug("Attempting to connect to Task1 Service...")
    try:
      rospy.wait_for_service('/task1_start', timeout=T1_TEST_TIME)
    except ((rospy.ROSException), e): 
      rospy.logdebug("Task1 Request failed - not advertised")
      return False
    return True


  def send_task1_request(self): 
    rospy.logdebug("Task1 Service connected. Sending request...")
    task1srv = rospy.ServiceProxy('/task1_start', Task1Service)
    task1_resp = task1srv()
    return task1_resp


  def stop_callback(self, event):
    self.trial_timeout=True
    return


  def start_trial_validation(self):
    success = self.prepare_for_task_request()
    rospy.sleep(rospy.Duration(1))
    if success: resp = self.send_task1_request()
    else: rospy.logerr("Task Request failed - not advertised")
    return


  # validation method, not for students
  def start_test_setup(self, validation_scenario):
    pass


  # validation method, not for students
  def start_test_validation(self, validation_scenario):
    pass    


class Task2(Task):
  def __init__(self, mode='coursework', validation_scenario=0):
    rospy.loginfo('================Starting Task2==============')
    Task.__init__(self,mode, validation_scenario)


  def spawn_trial_course(self):
    """ Spawns trial course - feel free to edit """
    # Despawn old objects 
    world_spawner.despawn_all(keyword='object', exceptions='golf')
    # self.goal_pt = world.randomize_goal()
    query_colour_name = list(BOX_COLORS.keys())[np.random.randint(0, len(BOX_COLORS.keys()))]
    self.query_color = BOX_COLORS[query_colour_name]


    self.box_locs = []
    
    # and spawn new ones
    ys = np.arange(T2_BOX_Y_LIMS[0], T2_BOX_Y_LIMS[1], step=TILE_SIDE)
    xs = np.arange(T2_BOX_X_LIMS[0], T2_BOX_X_LIMS[1], step=TILE_SIDE)
    for i, x_i in enumerate(xs):
      for j, y_j in enumerate(ys):
        if np.abs(x_i) < ROBOT_SAFETY_RADIUS_BOX and np.abs(y_j)< ROBOT_SAFETY_RADIUS_BOX : 
          continue
        self.box_locs.append([x_i,y_j])
      
    np.random.shuffle(self.box_locs)

    self.box_locs = self.box_locs[:T2_N_BOXES]

    # for testing: predefined box locations
    # self.box_locs = [ [0.4, -0.1], [0.4, 0], [0.4, 0.1] ]
    self.stack_goal_colours = []
    self.stack_goal_colours_rgb = []
    self.stack_goal_names = []

    self.goal_boxes = []

    for i, loc in enumerate(self.box_locs):

      # ensure one of each colour, then choose randomly
      selection = np.random.randint(0, len(BOX_COLORS.keys()))

      box_colour = list(BOX_COLORS.keys())[selection]
      mname = 'boxobject%02d_%s' % (i,box_colour)

      rotation_deg = np.random.randint(0, 90)
      rotation_rad = rotation_deg * (np.pi/ 180.0)

      if i < T2_STACK_SIZE:
        self.stack_goal_colours.append(box_colour)
        self.stack_goal_colours_rgb.append(BOX_COLORS[box_colour])
        self.stack_goal_names.append(mname)

      self.models[mname] = self.spawn_box_object(name=mname, color=box_colour ,
                                                 xlims = [loc[0],loc[0]], 
                                                 ylims = [loc[1],loc[1]],
                                                 z_rotation=rotation_rad)
      
    # define the goal stack location and rotation
    x = np.random.choice([0, 1])
    self.goal_loc = BASKET_LOCATIONS[x]
    self.goal_stack_rotation = (np.pi / 180.0) * np.random.randint(0, 90)

    return


  def reset_world(self): pass


  def prepare_for_task_request(self):
    rospy.logdebug("Attempting to connect to Task2 Service...")
    try:
      rospy.wait_for_service('/task2_start', timeout=T2_TEST_TIME)
    except ((rospy.ROSException), e): 
      rospy.logdebug("Task2 Request failed - not advertised")
      return False
    return True


  def send_task2_request(self): 
    rospy.logdebug("Task2 Service connected. Sending request...")
    task2srv = rospy.ServiceProxy('/task2_start', Task2Service)

    srv_request = Task2ServiceRequest()

    # save the colours we want in the request message
    for i in range(T2_STACK_SIZE):
      color_rgba = ColorRGBA(float(self.stack_goal_colours_rgb[i][0]),
                            float(self.stack_goal_colours_rgb[i][1]),
                            float(self.stack_goal_colours_rgb[i][2]),
                            1.0)
      srv_request.stack_colours.append(color_rgba)

    # define the goal stack position and rotation
    srv_request.stack_point.x = self.goal_loc[0]
    srv_request.stack_point.y = self.goal_loc[1]
    srv_request.stack_point.z = 0
    srv_request.stack_rotation = self.goal_stack_rotation

    # send the service request
    resp = task2srv(srv_request)

    return resp


  def stop_callback(self, event):
    self.trial_timeout=True
    return


  def start_trial_validation(self):
    success = self.prepare_for_task_request()
    if success: resp = self.send_task2_request()
    else: rospy.logerr("Task Request failed - not advertised")
    return

  # validation method, not for students
  def start_test_setup(self, validation_scenario):
    pass
  # validation method, not for students
  def start_test_validation(self, validation_scenario):
    pass


class Task3(Task):
  def __init__(self, mode='coursework', validation_scenario=0):
    rospy.loginfo('================Starting Task3==============')
    Task.__init__(self,mode, validation_scenario)


  def spawn_trial_course(self):
    """ Spawns trial course - feel free to edit """
    # Despawn old objects 
    world_spawner.despawn_all(keyword='object', exceptions='golf')
    # self.goal_pt = world.randomize_goal()
    query_colour_name = list(BOX_COLORS.keys())[np.random.randint(0, len(BOX_COLORS.keys()))]
    self.query_color = BOX_COLORS[query_colour_name]

    self.box_locs = []

    # get the edges of the tile grid
    tile_x_lims = [T3_BOX_X_LIMS[0] + T3_BOX_X_LIMS[0] % TILE_SIDE,
                   T3_BOX_X_LIMS[1] - T3_BOX_X_LIMS[1] % TILE_SIDE]
    tile_y_lims = [T3_BOX_Y_LIMS[0] + T3_BOX_Y_LIMS[0] % TILE_SIDE,
                   T3_BOX_Y_LIMS[1] - T3_BOX_Y_LIMS[1] % TILE_SIDE]

    # hardcode that there will be four obstacles, arranged in 4 tile spaces on the edges
    obstacle_y = [tile_y_lims[0] + TILE_SIDE / 2.,
                  np.random.uniform(low=tile_y_lims[0], high=tile_y_lims[1] - TILE_SIDE * 2.5),
                  tile_y_lims[1] - TILE_SIDE / 2.,
                  np.random.uniform(low=tile_y_lims[0] + TILE_SIDE * 2.5, high=tile_y_lims[1])]

    obstacle_x = [np.random.uniform(low=tile_x_lims[0], high=tile_x_lims[1] - TILE_SIDE * 2.5),
                  tile_x_lims[0] + TILE_SIDE / 2.,
                  np.random.uniform(low=tile_x_lims[0] + TILE_SIDE * 2.5, high=tile_x_lims[1]),
                  tile_x_lims[1] - TILE_SIDE / 2.]
    
    # and spawn new ones
    ys = np.arange(T3_BOX_Y_LIMS[0], T3_BOX_Y_LIMS[1], step=TILE_SIDE)
    xs = np.arange(T3_BOX_X_LIMS[0], T3_BOX_X_LIMS[1], step=TILE_SIDE)

    for i, x_i in enumerate(xs):

      for j, y_j in enumerate(ys):

        if np.abs(x_i) < ROBOT_SAFETY_RADIUS_BOX and np.abs(y_j)< ROBOT_SAFETY_RADIUS_BOX : 
          continue

        self.box_locs.append([x_i,y_j])

        # clear space around the obstacles
        for k in range(len(obstacle_x)):
          if x_i > obstacle_x[k] - 2*TILE_SIDE and x_i < obstacle_x[k] + 2*TILE_SIDE:
            if y_j > obstacle_y[k] - 2*TILE_SIDE and y_j < obstacle_y[k] + 2*TILE_SIDE:
              self.box_locs.pop()
              break
      
    np.random.shuffle(self.box_locs)

    # where will we put the target stack
    self.stack_location = self.box_locs[T3_N_BOXES]

    # clip for the rest of the boxes
    self.box_locs = self.box_locs[:T3_N_BOXES]

    self.stack_goal_colours = []
    self.stack_goal_colours_rgb = []
    self.stack_goal_names = []

    self.goal_boxes = []

    # place the cubes on the ground
    for i, loc in enumerate(self.box_locs):

      # ensure one of each colour, then choose randomly
      selection = np.random.randint(0, len(BOX_COLORS.keys()))

      box_colour = list(BOX_COLORS.keys())[selection]
      mname = 'boxobject%02d_%s' % (i,box_colour)

      rotation_deg = np.random.randint(0, 90)
      rotation_rad = rotation_deg * (np.pi/ 180.0)

      if i < T3_STACK_SIZE:
        self.stack_goal_colours.append(box_colour)
        self.stack_goal_colours_rgb.append(BOX_COLORS[box_colour])
        self.stack_goal_names.append(mname)

      self.models[mname] = self.spawn_box_object(name=mname, color=box_colour ,
                                                 xlims = [loc[0],loc[0]], 
                                                 ylims = [loc[1],loc[1]],
                                                 z_rotation=rotation_rad)

    # make the demo stack
    self.goal_stack_rotation = np.random.randint(0, 90) * (np.pi/ 180.0)
    cube_height = 45e-3
    cube_start = 45e-3
    for i, goal_colour in enumerate(self.stack_goal_colours):
      mname = 'object_stack_%02d' % (i)
      self.models[mname] = self.spawn_box_object(name=mname, color=goal_colour ,
                                                 xlims = [self.stack_location[0], self.stack_location[0]], 
                                                 ylims = [self.stack_location[1], self.stack_location[1]],
                                                 zlims = [cube_height * i + cube_start, cube_height * i + cube_start],
                                                 z_rotation=self.goal_stack_rotation)

    # define the goal stack location
    x = np.random.choice([0, 1])
    self.goal_loc = BASKET_LOCATIONS[x]

    self.obstacle_locs = []
    OBSTACLE_HEIGHT = 160e-3

    # add the obstacles to the scene
    for i in range(len(obstacle_x)):
      new_obs = Model(model_name="obstacle_1",
                      instance_name="obstacle_object_%i" % i,
                      model_type="sdf",
                      position=[obstacle_x[i], obstacle_y[i], OBSTACLE_HEIGHT / 2.0])
      world_spawner.spawn(new_obs)
      self.obstacle_locs.append([obstacle_x, obstacle_y])

    return


  def reset_world(self): pass


  def prepare_for_task_request(self):
    rospy.logdebug("Attempting to connect to Task3 Service...")
    try:
      rospy.wait_for_service('/task3_start', timeout=T3_TEST_TIME)
    except ((rospy.ROSException), e): 
      rospy.logdebug("Task3 Request failed - not advertised")
      return False
    return True


  def send_task3_request(self): 
    rospy.logdebug("Task3 Service connected. Sending request...")
    task3srv = rospy.ServiceProxy('/task3_start', Task3Service)

    srv_request = Task3ServiceRequest()

    # define the goal stack position and rotation
    srv_request.stack_point.x = self.goal_loc[0]
    srv_request.stack_point.y = self.goal_loc[1]
    srv_request.stack_point.z = 0
    srv_request.stack_rotation = self.goal_stack_rotation

    # send the service request
    resp = task3srv(srv_request)

    return resp


  def stop_callback(self, event):
    self.trial_timeout=True
    return


  def start_trial_validation(self):
    success = self.prepare_for_task_request()
    if success: resp = self.send_task3_request()
    else: rospy.logerr("Task Request failed - not advertised")
    return

  # validation method, not for students
  def start_test_setup(self, validation_scenario):
    pass

  # validation method, not for students
  def start_test_validation(self, validation_scenario):
    pass

# ----- Running the coursework node ----- #

def run_coursework3(mode):
  # run the coursework with the given mode:
  #   mode = 'coursework', for your use during development

  def handle_task_request(req):

    rospy.loginfo("enter handle_task_request")

    # Callback for selecting which task to start
    if req.task_index == 1:
      Task1(mode=mode)
    elif req.task_index == 2:
      Task2(mode=mode)
    elif req.task_index == 3:
      Task3(mode=mode)
    else:
      rospy.logwarn("Unrecognized task requested")

    rospy.loginfo("at end of handle_task_request")

    return TaskSetupResponse()

  # create the /task service callback
  s = rospy.Service('/task', TaskSetup, handle_task_request)
  rospy.loginfo("Ready to initiate task.")
  rospy.loginfo("Use rosservice call /task <INDEX> to start a task")
  rospy.spin()

if __name__ == "__main__":

  # create the world and run the coursework /task service
  rospy.init_node('coursework3_wrapper')
  world_spawner = WorldSpawner()
  world = World()
  run_coursework3(mode = 'coursework') # no need to change the mode

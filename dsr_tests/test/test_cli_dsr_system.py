import unittest
import pytest
import subprocess
import signal
import rclpy
import threading
import time
from dsr_msgs2.srv import *
import rclpy.duration
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import rclpy.type_support
import os

### !Note: Currently, real mode doesn't work. (TODO)
MODE = 'virtual'
NAMESPACE = "dsr01"
MODEL = 'm1013'
SRV_CALL_TIMEOUT = 30	
PORT = 12345

RVIZ = True
START_EMULATOR = True 



### In case of cloud ci pipeline, 
### we don't need turn on rviz and 
### emulator should be already setup at pipeline configuration.
if os.getenv("TEST_ON_CLOUD", "false") == "true":
	print("START TEST CODE ON CLOUD ENVIRONMENT !!")
	RVIZ = False
	START_EMULATOR = False
	

@pytest.fixture(scope='session', autouse=True)
def bringUp():
	print("Setting up class fixture")
	rclpy.init()
	# TODO: handle node only once over classes.
	# node = rclpy.create_node("dsr_move_test_node", namespace=NAMESPACE)
	bringup_script = subprocess.Popen([ # how to align RAII?
			"ros2", "launch", "dsr_tests",
			"dsr_bringup_without_spawner_test.launch.py",
			"mode:={}".format(MODE),
			"name:={}".format(NAMESPACE),
			"port:={}".format(PORT),
			"rviz:={}".format(RVIZ),
			"start_emulator:={}".format(START_EMULATOR)
			],
			stdout=subprocess.PIPE,
			stderr=subprocess.STDOUT,
			universal_newlines=True
		)
	spawner_script = subprocess.Popen([
			"ros2", "launch", "dsr_tests",
			"dsr_spawner_cli_test.launch.py",
			"name:={}".format(NAMESPACE)
			],
			stdout=subprocess.PIPE,
			stderr=subprocess.STDOUT,
			universal_newlines=True
		)

	# Assume if spawners are successfully loaded, Prerequisite done.
	try:
		(stdout, _ ) = spawner_script.communicate(timeout=30)
		if 2 != stdout.count('\033[92m'+"Configured and activated"):
			bringup_script.send_signal(signal.SIGINT)
			spawner_script.send_signal(signal.SIGINT)
			raise Exception('Failed Loading Spawner. stdout : {}, \nAll tests will be failed !!!'
							.format(stdout))
	except subprocess.TimeoutExpired as e:
		bringup_script.send_signal(signal.SIGINT)
		spawner_script.send_signal(signal.SIGINT)
		raise Exception('Spawner Time out !!, All tests will be failed !!!!')
	
	print("Bringup successfully done.")
	yield 

	spawner_script.send_signal(signal.SIGINT)
	bringup_script.send_signal(signal.SIGINT)
	rclpy.shutdown()
	print("Bringup terminated.")


""" Motion Client Test Class """
class TestDsrMoveCli(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		print("=========================================")
		print("======= DooSan Motion Tests Start =======")
		cls._lock = threading.Lock()
		cls.node = rclpy.create_node("test_dsr_move_node", namespace=NAMESPACE)

	@classmethod
	def tearDownClass(cls):
		print("======= DooSan Motion Tests Finished ====")
		print("=========================================")

	def setUp(self):
		TestDsrMoveCli._lock.acquire()
		print("Ready For Motion Test!!")
		self._set_ready_pose()
		
	def tearDown(self):
		print("Clear For Motion Test!!")
		self._set_home_pose() # to avoid singularity
		TestDsrMoveCli._lock.release()

	def _set_ready_pose(self):
		""" Move Joint to Ready Position """
		cli = self.node.create_client(MoveJoint, "motion/move_joint")
		target_pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
		req = MoveJoint.Request()
		req.pos = target_pos
		req.time = 5.0
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_joint future task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_joint response")
		self.node.destroy_client(cli)

		## READY CHECK
		cli = self.node.create_client(GetCurrentPose, "system/get_current_pose")
		## TODO: fix hard code section 
		# (replace 'space_type=0' with 'space_type=Request.ROBOT_SPACE_JOINT')
		req = GetCurrentPose.Request(space_type=0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "system/get_current_pose task")
		resp = future.result()
		self.assertTrue(resp.success == True, "system/get_current_pose response")
		self.node.destroy_client(cli)

	def _set_home_pose(self):
		""" Move Joint to Ready Position """
		cli = self.node.create_client(MoveJoint, "motion/move_joint")
		target_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		req = MoveJoint.Request()
		req.pos = target_pos
		req.time = 5.0
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_joint future task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_joint response")
		self.node.destroy_client(cli)

		## READY CHECK
		cli = self.node.create_client(GetCurrentPose, "system/get_current_pose")
		## TODO: fix hard code section 
		# (replace 'space_type=0' with 'space_type=Request.ROBOT_SPACE_JOINT')
		req = GetCurrentPose.Request(space_type=0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "system/get_current_pose task")
		resp = future.result()
		self.assertTrue(resp.success == True, "system/get_current_pose response")
		self.node.destroy_client(cli)

	""" Motion Client """
	# Move Joint Test 
	def test_move_joint_cli(self):
		print("Move Joint Client Test are starting...") # Debug
		cli = self.node.create_client(MoveJoint, "motion/move_joint")
		target_pos = [0.0, 0.0, 45.0, 0.0, 45.0, 0.0]
		req = MoveJoint.Request()
		req.pos = target_pos
		req.time = 5.0
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_joint future task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_joint response")
		self.node.destroy_client(cli)

		cli = self.node.create_client(GetCurrentPose, "system/get_current_pose")
		## TODO: fix hard code section 
		# (replace 'space_type=0' with 'space_type=Request.ROBOT_SPACE_JOINT')
		req = GetCurrentPose.Request(space_type=0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "system/get_current_pose task")
		resp = future.result()
		self.assertTrue(resp.success == True, "system/get_current_pose response")
		self.node.destroy_client(cli)

		## Check pose from sensor stream literally equals to target.
		self.assertAlmostEqual(target_pos[0], resp.pos[0], delta=0.001)
		self.assertAlmostEqual(target_pos[1], resp.pos[1], delta=0.001)
		self.assertAlmostEqual(target_pos[2], resp.pos[2], delta=0.001)
		self.assertAlmostEqual(target_pos[3], resp.pos[3], delta=0.001)
		self.assertAlmostEqual(target_pos[4], resp.pos[4], delta=0.001)
		self.assertAlmostEqual(target_pos[5], resp.pos[5], delta=0.001)

	# Move JointX Test
	def test_move_jointx_cli(self):
		print("Move Jointx Client Test are starting...") # Debug
		self._set_ready_pose()

		""" Move Line to Ready Position """
		cli = self.node.create_client(MoveLine, "motion/move_line")
		target_pos = [400.0, 500.0, 500.0, 0.0, 180.0, 0.0]
		req = MoveLine.Request(pos=target_pos, time=5.0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_line future task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_line response")
		self.node.destroy_client(cli)

		""" Get Current PosX"""
		cli = self.node.create_client(GetCurrentPosx, "aux_control/get_current_posx")
		req = GetCurrentPosx.Request(ref=0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "aux_control/get_current_posx task")
		resp = future.result()
		self.assertTrue(resp.success == True, "aux_control/get_current_posx response")
		self.node.destroy_client(cli)

		""" Move JointX """
		cli = self.node.create_client(MoveJointx, "motion/move_jointx")
		target_pos = [400.0, 500.0, 800.0, 0.0, 180.0, 0.0]
		req = MoveJointx.Request()
		req.pos = target_pos
		req.vel = 30.0
		req.acc = 60.0
		req.sol = int(resp.task_pos_info[0].data[6])
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_jointx future task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_jointx response")
		self.node.destroy_client(cli)

		""" Get Current PosX"""
		cli = self.node.create_client(GetCurrentPosx, "aux_control/get_current_posx")
		req = GetCurrentPosx.Request(ref=0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "aux_control/get_current_posx task")
		resp = future.result()
		self.assertTrue(resp.success == True, "aux_control/get_current_posx response")
		self.node.destroy_client(cli)

		## Check pose from sensor stream literally equals to target.
		current_pose = resp.task_pos_info[0].data

		self.assertAlmostEqual(target_pos[0], current_pose[0], delta=0.001)
		self.assertAlmostEqual(target_pos[1], current_pose[1], delta=0.001)
		self.assertAlmostEqual(target_pos[2], current_pose[2], delta=0.001)
		self.assertAlmostEqual(target_pos[4], current_pose[4], delta=0.001)


	## TODO: Move home service occurs asynchronously. need to update logic to wait.
	# Move Home Test 
	# def test_move_home_cli(self):
	# 	print("Move Home Client Test are starting...") # Debug
	# 	""" Move Joint to Ready Position """
	# 	cli = self.node.create_client(MoveJoint, "motion/move_joint")
	# 	target_pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
	# 	req = MoveJoint.Request()
	# 	req.pos = target_pos
	# 	# req.sync_type = 1 # ASYNC
	# 	req.time = 5.0
	# 	future = cli.call_async(req)
	# 	rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
	# 	self.assertTrue(future.done(), "motion/move_joint future task")
	# 	resp = future.result()
	# 	self.assertTrue(resp.success == True, "motion/move_joint response")
	# 	self.node.destroy_client(cli)
		
	# 	## READY CHECK
	# 	cli = self.node.create_client(GetCurrentPose, "system/get_current_pose")
	# 	## TODO: fix hard code section 
	# 	# (replace 'space_type=0' with 'space_type=Request.ROBOT_SPACE_JOINT')
	# 	req = GetCurrentPose.Request(space_type=0)
	# 	future = cli.call_async(req)
	# 	rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
	# 	self.assertTrue(future.done(), "system/get_current_pose task")
	# 	resp = future.result()
	# 	self.assertTrue(resp.success == True, "system/get_current_pose response")
	# 	self.node.destroy_client(cli)

	# 	## Check pose from sensor stream literally equals to target.
		
	# 	self.assertAlmostEqual(target_pos[0], resp.pos[0], delta=0.001)
	# 	self.assertAlmostEqual(target_pos[1], resp.pos[1], delta=0.001)
	# 	self.assertAlmostEqual(target_pos[2], resp.pos[2], delta=0.001)
	# 	self.assertAlmostEqual(target_pos[3], resp.pos[3], delta=0.001)
	# 	self.assertAlmostEqual(target_pos[4], resp.pos[4], delta=0.001)
	# 	self.assertAlmostEqual(target_pos[5], resp.pos[5], delta=0.001)

	# 	""" Move Home """
	# 	home_cli = self.node.create_client(MoveHome, "motion/move_home")
	# 	home_req = MoveHome.Request()
	# 	home_req.target = 0 
	# 	home_future = home_cli.call_async(home_req)
	# 	rclpy.spin_until_future_complete(self.node, home_future, timeout_sec=SRV_CALL_TIMEOUT)
	# 	self.assertTrue(home_future.done(), "motion/move_home future task")
	# 	home_resp = home_future.result()
	# 	self.assertTrue(home_resp.success == True, "motion/move_home reponse")
	# 	self.node.destroy_client(home_cli)

	# 	""" Check Event !!""" 
	# 	start_time = self.node.get_clock.now()
	# 	duration = rclpy.duration.Duration(seconds=10)

	# 	# while (self.node.get_clock.now() - start_time) < duration : 

	# 	""" Current Pose """
	# 	cur_cli = self.node.create_client(GetCurrentPose, "system/get_current_pose")
	# 	cur_req = GetCurrentPose.Request()
	# 	cur_req.space_type = 0
	# 	cur_future = cur_cli.call_async(cur_req)
	# 	rclpy.spin_until_future_complete(self.node, cur_future, timeout_sec=SRV_CALL_TIMEOUT)
	# 	self.assertTrue(cur_future.done(), "system/get_current_posej")
	# 	cur_resp = cur_future.result()
	# 	self.assertTrue(cur_resp.success == True, "system/get_current_posej")
	# 	self.node.destroy_client(cur_cli)

	# 	## Check pose from sensor stream literally equals to target.
	# 	self.assertAlmostEqual(0, cur_resp.pos[0], delta=0.001)
	# 	self.assertAlmostEqual(0, cur_resp.pos[1], delta=0.001)
	# 	self.assertAlmostEqual(0, cur_resp.pos[2], delta=0.001)
	# 	self.assertAlmostEqual(0, cur_resp.pos[3], delta=0.001)
	# 	self.assertAlmostEqual(0, cur_resp.pos[4], delta=0.001)
	# 	self.assertAlmostEqual(0, cur_resp.pos[5], delta=0.001)

	
	# Move Line Test 
	def test_move_line_cli(self):
		print("Move Line Client Test are starting...") # Debug
		cli = self.node.create_client(MoveJoint, "motion/move_joint")
		target_pos = [0, 0, 90, 0, 90, 0]
		req = MoveJoint.Request(pos=target_pos, vel=30., acc=30.)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_joint future task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_joint response")
		self.node.destroy_client(cli)

		""" Current Pose """
		cli = self.node.create_client(GetCurrentPose, "system/get_current_pose")
		## TODO: fix hard code section 
		# (replace 'space_type=0' with 'space_type=Request.ROBOT_SPACE_JOINT')
		req = GetCurrentPose.Request(space_type=0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "system/get_current_pose task")
		resp = future.result()
		self.assertTrue(resp.success == True, "system/get_current_pose response")
		self.node.destroy_client(cli)

		## Check pose from sensor stream literally equals to target.
		
		self.assertAlmostEqual(target_pos[0], resp.pos[0], delta=0.001)
		self.assertAlmostEqual(target_pos[1], resp.pos[1], delta=0.001)
		self.assertAlmostEqual(target_pos[2], resp.pos[2], delta=0.001)
		self.assertAlmostEqual(target_pos[3], resp.pos[3], delta=0.001)
		self.assertAlmostEqual(target_pos[4], resp.pos[4], delta=0.001)
		self.assertAlmostEqual(target_pos[5], resp.pos[5], delta=0.001)

		cli = self.node.create_client(MoveLine, "motion/move_line")
		target_pos = [400, 500, 800, 0, 180, 0]
		req = MoveLine.Request(pos=target_pos, time=5.0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_line future task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_line response")
		self.node.destroy_client(cli)

		cli = self.node.create_client(GetCurrentPosx, "aux_control/get_current_posx")
		req = GetCurrentPosx.Request(ref=0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "aux_control/get_current_posx task")
		resp = future.result()
		self.assertTrue(resp.success == True, "aux_control/get_current_posx response")
		self.node.destroy_client(cli)

		## Check pose from sensor stream literally equals to target.
		current_pose = resp.task_pos_info[0].data

		self.assertAlmostEqual(target_pos[0], current_pose[0], delta=0.001)
		self.assertAlmostEqual(target_pos[1], current_pose[1], delta=0.001)
		self.assertAlmostEqual(target_pos[2], current_pose[2], delta=0.001)
		self.assertAlmostEqual(target_pos[4], current_pose[4], delta=0.001)


	# Move Circle Test 
	def test_move_circle_cli(self):
		print("Move Circle Client Test are starting...") # Debug
		cli = self.node.create_client(MoveJoint, "motion/move_joint")
		target_pos = [0, 0, 90, 0, 90, 0]
		req = MoveJoint.Request(pos=target_pos, vel=30., acc=30.)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_joint future task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_joint response")
		self.node.destroy_client(cli)

		pos_list = []
		cli = self.node.create_client(MoveCircle, "motion/move_circle")

		""" Target Pose 1"""
		target_pos1 = Float64MultiArray()
		# Layout Setting
		dim = MultiArrayDimension()
		dim.label = "columns"
		dim.size = 6
		dim.stride = 6
		target_pos1.layout.dim = [dim]
		target_pos1.layout.data_offset = 0
		target_pos1.data = [400.0, 500.0, 800.0, 0.0, 180.0, 0.0]
		pos_list.append(target_pos1)

		""" Target Pose 2"""
		target_pos2 = Float64MultiArray()
		# Layout Setting
		target_pos2.layout.dim = [dim]
		target_pos2.layout.data_offset = 0
		target_pos2.data = [400.0, 500.0, 500.0, 0.0, 180.0, 0.0]
		pos_list.append(target_pos2)

		req = MoveCircle.Request()
		req.pos = pos_list
		req.time = 5.0
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_circle task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_circle response")
		self.node.destroy_client(cli)

		""" Current Pose """
		cli = self.node.create_client(GetCurrentPosx, "aux_control/get_current_posx")
		req = GetCurrentPosx.Request(ref=0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "aux_control/get_current_posx task")
		resp = future.result()
		self.assertTrue(resp.success == True, "aux_control/get_current_posx response")
		self.node.destroy_client(cli)

		## Check pose from sensor stream literally equals to target.
		current_pose = resp.task_pos_info[0].data
		self.assertAlmostEqual(target_pos2.data[0], current_pose[0], delta=0.001)
		self.assertAlmostEqual(target_pos2.data[1], current_pose[1], delta=0.001)
		self.assertAlmostEqual(target_pos2.data[2], current_pose[2], delta=0.001)
		self.assertAlmostEqual(target_pos2.data[4], current_pose[4], delta=0.001)


	# Move Spline Joint Test
	def test_move_spline_joint_cli(self):
		self._set_home_pose()
		print("Move Spline Joint Client Test are starting...") # Debug
		""" Move Spline Joint """
		pos_list = []
		cli = self.node.create_client(MoveSplineJoint, "motion/move_spline_joint")

		""" Target Pose 1 """
		target_pos1 = Float64MultiArray()
		# Layout Setting
		dim = MultiArrayDimension()
		dim.label = "columns"
		dim.size = 6
		dim.stride = 6
		target_pos1.layout.dim = [dim]
		target_pos1.layout.data_offset = 0
		target_pos1.data = [10.0, -10.0, 20.0, -30.0, 10.0, 20.0]
		pos_list.append(target_pos1)

		""" Target Pose 2 """
		target_pos2 = Float64MultiArray()
		# Layout Setting
		target_pos2.layout.dim = [dim]
		target_pos2.layout.data_offset = 0
		target_pos2.data = [25.0, 0.0, 10.0, -50.0, 20.0, 40.0]
		pos_list.append(target_pos2)

		""" Target Pose 3 """
		target_pos3 = Float64MultiArray()
		# Layout Setting
		target_pos3.layout.dim = [dim]
		target_pos3.layout.data_offset = 0
		target_pos3.data = [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
		pos_list.append(target_pos3)

		""" Target Pose 4 """
		target_pos4 = Float64MultiArray()
		# Layout Setting
		target_pos4.layout.dim = [dim]
		target_pos4.layout.data_offset = 0
		target_pos4.data = [30.0, 10.0, 30.0, -20.0, 10.0, 60.0]
		pos_list.append(target_pos4)

		""" Target Pose 5 """
		target_pos5 = Float64MultiArray()
		# Layout Setting
		target_pos5.layout.dim = [dim]
		target_pos5.layout.data_offset = 0
		target_pos5.data = [20.0, 20.0, 40.0, 20.0, 0.0, 90.0]
		pos_list.append(target_pos5)

		req = MoveSplineJoint.Request()
		req.pos = pos_list
		req.pos_cnt = 5
		req.time = 10.0
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_spline_joint task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_spline_joint response")
		self.node.destroy_client(cli)

		""" Current Pose """
		cli = self.node.create_client(GetCurrentPose, "system/get_current_pose")
		## TODO: fix hard code section 
		# (replace 'space_type=0' with 'space_type=Request.ROBOT_SPACE_JOINT')
		req = GetCurrentPose.Request(space_type=0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "system/get_current_pose task")
		resp = future.result()
		self.assertTrue(resp.success == True, "system/get_current_pose response")
		self.node.destroy_client(cli)

		## Check pose from sensor stream literally equals to target.
		last_pos = target_pos5.data
		self.assertAlmostEqual(last_pos[0], resp.pos[0], delta=0.001)
		self.assertAlmostEqual(last_pos[1], resp.pos[1], delta=0.001)
		self.assertAlmostEqual(last_pos[2], resp.pos[2], delta=0.001)
		self.assertAlmostEqual(last_pos[3], resp.pos[3], delta=0.001)
		self.assertAlmostEqual(last_pos[4], resp.pos[4], delta=0.001)
		self.assertAlmostEqual(last_pos[5], resp.pos[5], delta=0.001)

	
	# Move Spline Task Test
	def test_move_spline_task_cli(self):
		print("Move Spline Task Client Test are starting...") # Debug
		""" Move Line """
		cli = self.node.create_client(MoveLine, "motion/move_line")
		target_pos = [600, 43, 500, 0, 180, 0]
		req = MoveLine.Request(pos=target_pos, time=5.0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_line future task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_line response")
		self.node.destroy_client(cli)

		""" Move Spline Task """
		pos_list = []
		cli = self.node.create_client(MoveSplineTask, "motion/move_spline_task")

		""" Target Pose 1 """
		target_pos1 = Float64MultiArray()
		# Layout Setting
		dim = MultiArrayDimension()
		dim.label = "columns"
		dim.size = 6
		dim.stride = 6
		target_pos1.layout.dim = [dim]
		target_pos1.layout.data_offset = 0
		target_pos1.data = [600.0, 600.0, 600.0, 0.0, 175.0, 0.0]
		pos_list.append(target_pos1)

		""" Target Pose 2 """
		target_pos2 = Float64MultiArray()
		# Layout Setting
		target_pos2.layout.dim = [dim]
		target_pos2.layout.data_offset = 0
		target_pos2.data = [600.0, 750.0, 600.0, 0.0, 175.0, 0.0]
		pos_list.append(target_pos2)

		""" Target Pose 3 """
		target_pos3 = Float64MultiArray()
		# Layout Setting
		target_pos3.layout.dim = [dim]
		target_pos3.layout.data_offset = 0
		target_pos3.data = [150.0, 600.0, 450.0, 0.0, 175.0, 0.0]
		pos_list.append(target_pos3)


		""" Target Pose 4 """
		target_pos4 = Float64MultiArray()
		# Layout Setting
		target_pos4.layout.dim = [dim]
		target_pos4.layout.data_offset = 0
		target_pos4.data = [-300.0, 300.0, 300.0, 0.0, 175.0, 0.0]
		pos_list.append(target_pos4)

		""" Target Pose 5 """
		target_pos5 = Float64MultiArray()
		# Layout Setting
		target_pos5.layout.dim = [dim]
		target_pos5.layout.data_offset = 0
		target_pos5.data = [-200.0, 700.0, 500.0, 0.0, 175.0, 0.0]
		pos_list.append(target_pos5)

		""" Target Pose 6 """
		target_pos6 = Float64MultiArray()
		# Layout Setting
		target_pos6.layout.dim = [dim]
		target_pos6.layout.data_offset = 0
		target_pos6.data = [600.0, 600.0, 400.0, 0.0, 175.0, 0.0]
		pos_list.append(target_pos6)

		req = MoveSplineTask.Request()
		req.pos = pos_list
		req.pos_cnt = 6
		req.time = 20.0
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_spline_joint task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_spline_joint response")
		self.node.destroy_client(cli)

		""" Current Pose """
		cli = self.node.create_client(GetCurrentPosx, "aux_control/get_current_posx")
		req = GetCurrentPosx.Request(ref=0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "aux_control/get_current_posx task")
		resp = future.result()
		self.assertTrue(resp.success == True, "aux_control/get_current_posx response")
		self.node.destroy_client(cli)

		## Check pose from sensor stream literally equals to target.
		current_pose = resp.task_pos_info[0].data
		self.assertAlmostEqual(target_pos6.data[0], current_pose[0], delta=0.001)
		self.assertAlmostEqual(target_pos6.data[1], current_pose[1], delta=0.001)
		self.assertAlmostEqual(target_pos6.data[2], current_pose[2], delta=0.001)
		self.assertAlmostEqual(target_pos6.data[4], current_pose[4], delta=0.001)
	

	""" TODO : How can check the motion correct?"""
	# Move Spiral Test
	def test_move_spiral_cli(self):
		print("Move Spiral Client Test are starting...") # Debug
		""" Move Spiral """
		cli = self.node.create_client(MoveSpiral, "motion/move_spiral")
		req = MoveSpiral.Request()
		req.revolution = 5.0
		req.max_radius = 20.0
		req.max_length = 10.0
		req.time = 10.0
		req.task_axis = 2
		req.ref = 1
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "motion/move_spiral future task")
		resp = future.result()
		self.assertTrue(resp.success == True, "motion/move_spiral response")
		self.node.destroy_client(cli)


	# Move Pause and Resume Test
	def test_move_pause_and_resume_cli(self):
		self._set_home_pose()
		print("Move Pause and Resume Client Test are starting...") # Debug

		""" Move Joint """
		move_cli = self.node.create_client(MoveJoint, "motion/move_joint")
		target_pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
		move_req = MoveJoint.Request()
		move_req.pos = target_pos
		move_req.time = 5.0
		move_req.sync_type = 1
		move_future = move_cli.call_async(move_req)
		rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(move_future.done(), "motion/move joint future task")
		move_resp = move_future.result()
		self.assertTrue(move_resp.success == True, "motion/move joint response")
		self.node.destroy_client(move_cli)

		""" Move Pause """
		pause_cli = self.node.create_client(MovePause, "motion/move_pause")
		pause_req = MovePause.Request()
		pause_future = pause_cli.call_async(pause_req)
		rclpy.spin_until_future_complete(self.node, pause_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(pause_future.done(), "motion/move_pause future task")
		pause_resp = pause_future.result()
		self.assertTrue(pause_resp.success == True, "motion/move_pause response")
		self.node.destroy_client(pause_cli)

		""" Check Motion """
		check_cli = self.node.create_client(CheckMotion, "motion/check_motion")
		check_req = CheckMotion.Request()
		check_future = check_cli.call_async(check_req)
		rclpy.spin_until_future_complete(self.node, check_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(check_future.done(), "motion/check_motion future task")
		check_resp = check_future.result()
		self.assertTrue((check_resp.status == 2) and (check_resp.success == True) ,"motion/check_motion response") # 2 : motion in operation
		self.node.destroy_client(check_cli)

		""" Move Resume """
		resume_cli = self.node.create_client(MoveResume, "motion/move_resume")
		resume_req = MoveResume.Request()
		resume_future = resume_cli.call_async(resume_req)
		rclpy.spin_until_future_complete(self.node, resume_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(resume_future.done(), "motion/move_resume future task")
		resume_resp = resume_future.result()
		self.assertTrue(resume_resp.success == True, "motion/move_resume response")
		self.node.destroy_client(pause_cli)

		""" Check Motion """
		check_cli = self.node.create_client(CheckMotion, "motion/check_motion")
		check_req = CheckMotion.Request()
		check_future = check_cli.call_async(check_req)
		rclpy.spin_until_future_complete(self.node, check_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(check_future.done(), "motion/check_motion future task")
		check_resp = check_future.result()
		self.assertTrue((check_resp.status == 2) and (check_resp.success == True) ,"motion/check_motion response") # 2 : motion in operation
		self.node.destroy_client(check_cli)


	# Move Stop Test 
	def test_move_stop_cli(self):
		print("Move Stop Client Test are starting...") # Debug

		""" Move Joint """
		move_cli = self.node.create_client(MoveJoint, "motion/move_joint")
		target_pos = [0.0, 0.0, 45.0, 0.0, 45.0, 0.0]
		move_req = MoveJoint.Request()
		move_req.pos = target_pos
		move_req.time = 5.0
		move_req.sync_type = 1
		move_future = move_cli.call_async(move_req)

		""" Motion Stop """
		stop_cli = self.node.create_client(MoveStop, "motion/move_stop")
		stop_req = MoveStop.Request()
		stop_req.stop_mode = 0
		stop_future = stop_cli.call_async(stop_req)
		rclpy.spin_until_future_complete(self.node, stop_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(stop_future.done(), "motion/move_stop future task")
		pause_resp = stop_future.result()
		self.assertTrue(pause_resp.success == True, "motion/move_stop response")
		self.node.destroy_client(stop_future)

		""" Check Motion """
		check_cli = self.node.create_client(CheckMotion, "motion/check_motion")
		check_req = CheckMotion.Request()
		check_future = check_cli.call_async(check_req)
		rclpy.spin_until_future_complete(self.node, check_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(check_future.done(), "motion/check_motion future task")
		check_resp = check_future.result()
		# Motion Status is 0
		self.assertTrue((check_resp.status == 0) and (check_resp.success == True) ,"motion/check_motion response")
		self.node.destroy_client(check_cli)

		""" Check Pose """
		cli = self.node.create_client(GetCurrentPose, "system/get_current_pose")
	 	## TODO: fix hard code section 
		# (replace 'space_type=0' with 'space_type=Request.ROBOT_SPACE_JOINT')
		req = GetCurrentPose.Request(space_type=0)
		future = cli.call_async(req)
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(future.done(), "system/get_current_pose task")
		resp = future.result()
		self.assertTrue(resp.success == True, "system/get_current_pose response")
		self.node.destroy_client(cli)

		## Check pose from sensor stream literally equals to target.
		self.assertTrue(target_pos[2] != resp.pos[2])
		self.assertTrue(target_pos[4] != resp.pos[4])

	

	###!note: self.node.create_client(MoveWait, "motion/move_wait") TEST FAILURE !!
	#### desc) waiting between previous motion and next motion. -> no request field
	# Move Wait Test
	# def test_move_wait_cli(self):
		# """ Move Joint """
		# move_cli = self.node.create_client(MoveJoint, "motion/move_joint")
		# target_pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
		# move_req = MoveJoint.Request()
		# move_req.pos = target_pos
		# move_req.time = 5.0
		# move_req.sync_type = 1
		# move_future = move_cli.call_async(move_req)
		# rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=SRV_CALL_TIMEOUT)
		# self.assertTrue(move_future.done(), "motion/move joint future task")
		# move_resp = move_future.result()
		# self.assertTrue(move_resp.success == True, "motion/move joint response")
		# self.node.destroy_client(move_cli)

		# """ Motion Wait """
		# wait_cli = self.node.create_client(MoveWait, "motion/move_wait")
		# wait_req = MoveWait.Request()
		# wait_future = wait_cli.call_async(wait_req)
		# rclpy.spin_until_future_complete(self.node, wait_future, timeout_sec=SRV_CALL_TIMEOUT)


		# """ Check Pose """
		# cli = self.node.create_client(GetCurrentPose, "system/get_current_pose")
	 	# ## TODO: fix hard code section 
		# # (replace 'space_type=0' with 'space_type=Request.ROBOT_SPACE_JOINT')
		# req = GetCurrentPose.Request(space_type=0)
		# future = cli.call_async(req)
		# rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
		# self.assertTrue(future.done(), "system/get_current_pose task")
		# resp = future.result()
		# self.assertTrue(resp.success == True, "system/get_current_pose response")
		# self.node.destroy_client(cli)

		# ## Check pose from sensor stream literally equals to target.
		# self.assertAlmostEqual(target_pos[0], resp.pos[0], delta=0.001)
		# self.assertAlmostEqual(target_pos[1], resp.pos[1], delta=0.001)
		# self.assertAlmostEqual(target_pos[2], resp.pos[2], delta=0.001)
		# self.assertAlmostEqual(target_pos[3], resp.pos[3], delta=0.001)
		# self.assertAlmostEqual(target_pos[4], resp.pos[4], delta=0.001)
		# self.assertAlmostEqual(target_pos[5], resp.pos[5], delta=0.001)

	
	# Move Periodic Test  
	def test_move_periodic_cli(self):
		print("Move Periodic Client Test are starting...") # Debug

		""" Move Perioidic """
		period_cli = self.node.create_client(MovePeriodic, "motion/move_periodic")
		period_req = MovePeriodic.Request()
		period_req.amp = [10.0, 0.0, 0.0, 0.0, 30.0, 0.0]
		period_req.periodic = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
		period_req.acc = 0.2
		period_req.repeat = 5
		period_req.ref = 1
		period_future = period_cli.call_async(period_req)
		rclpy.spin_until_future_complete(self.node, period_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(period_future.done(), "motion/move_periodic future task")
		periodic_resp = period_future.result()
		self.assertTrue(periodic_resp.success == True, "motion/move_stop response")
		self.node.destroy_client(period_cli)


	# # Move Blending Test
	def test_move_blending_cli(self):
		print("Move Blending Client Test are starting...") # Debug

		""" Move Joint for Initial Angle """
		move_cli = self.node.create_client(MoveJoint, "motion/move_joint")
		target_pos = [45.0, 0.0, 90.0, 0.0, 90.0, 45.0]
		move_req = MoveJoint.Request()
		move_req.pos = target_pos
		move_req.vel = 30.0
		move_req.acc = 60.0
		move_future = move_cli.call_async(move_req)
		rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(move_future.done(), "motion/move joint future task")
		move_resp = move_future.result()
		self.assertTrue(move_resp.success == True, "motion/move joint response")
		self.node.destroy_client(move_cli)

		""" Get Current Posj """
		get_current_posj_cli = self.node.create_client(GetCurrentPosj, "aux_control/get_current_posj")
		get_current_posj_future = get_current_posj_cli.call_async(GetCurrentPosj.Request())
		rclpy.spin_until_future_complete(self.node, get_current_posj_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_current_posj_future.done(), "aux_control/get_current_posj service working is not done.")
		get_current_posj_resp = get_current_posj_future.result()
		self.assertTrue(get_current_posj_resp.success == True, "aux_control/get_current_posj service is not working correctly.")
		self.node.destroy_client(get_current_posj_cli)
		
		get_current_posj_result = get_current_posj_resp.pos
		# print(get_current_posj_result)
		self.assertAlmostEqual(get_current_posj_result[0], target_pos[0], delta = 0.1)
		self.assertAlmostEqual(get_current_posj_result[1], target_pos[1], delta = 0.1)
		self.assertAlmostEqual(get_current_posj_result[2], target_pos[2], delta = 0.1)
		self.assertAlmostEqual(get_current_posj_result[3], target_pos[3], delta = 0.1)
		self.assertAlmostEqual(get_current_posj_result[4], target_pos[4], delta = 0.1)
		self.assertAlmostEqual(get_current_posj_result[5], target_pos[5], delta = 0.1)

		
		if(move_resp.success == True):
			""" Move Line for Initial Position """
			move_line_cli = self.node.create_client(MoveLine, "motion/move_line")
			move_line_req = MoveLine.Request()
			move_line_req.pos = [370.0, 420.0, 650.0, 0.0, 180.0, 0.0]
			move_line_req.vel = [150.0, 150.0]
			move_line_req.acc = [250.0, 250.0]
			move_line_future = move_line_cli.call_async(move_line_req)
			rclpy.spin_until_future_complete(self.node, move_line_future, timeout_sec=SRV_CALL_TIMEOUT)
			self.assertTrue(move_line_future.done(), "motion/move_line service working is not done.")
			move_line_resp = move_line_future.result()
			self.assertTrue(move_line_resp.success == True, "motion/move_line service is not working correctly.")
			self.node.destroy_client(move_line_cli)

			""" Get Current Posx """
			get_current_posx_cli = self.node.create_client(GetCurrentPosx, "aux_control/get_current_posx")
			get_current_posx_future = get_current_posx_cli.call_async(GetCurrentPosx.Request())
			rclpy.spin_until_future_complete(self.node, get_current_posx_future, timeout_sec=SRV_CALL_TIMEOUT)
			self.assertTrue(get_current_posx_future.done(), "aux_control/get_current_posj service working is not done.")
			get_current_posx_resp = get_current_posx_future.result()
			self.assertTrue(get_current_posx_resp.success == True, "aux_control/get_current_posj service is not working correctly.")
			self.node.destroy_client(get_current_posx_cli)
			
			get_current_posx_result = get_current_posx_resp.task_pos_info[0].data
			# print(get_current_posx_result)
			self.assertAlmostEqual(get_current_posx_result[0], move_line_req.pos[0], delta = 0.1)
			self.assertAlmostEqual(get_current_posx_result[1], move_line_req.pos[1], delta = 0.1)
			self.assertAlmostEqual(get_current_posx_result[2], move_line_req.pos[2], delta = 0.1)
			self.assertAlmostEqual(get_current_posx_result[4], move_line_req.pos[4], delta = 0.1)

			if(move_line_resp.success == True):
				""" Move Blending """
				move_blend_cli = self.node.create_client(MoveBlending, "motion/move_blending")
				move_blend_req = MoveBlending.Request()
				move_blend_req.vel = [150.0, 150.0]
				move_blend_req.acc = [250.0, 250.0]
				move_blend_req.ref = 0
				move_blend_req.mode = 0
				move_blend_req.sync_type = 0
				move_blend_req.pos_cnt = 5

				""" Segment Pose 1 """
				segment_pos = []
				segment_pos1 = Float64MultiArray()
				# Layout Setting
				dim = MultiArrayDimension()
				dim.label = "columns"
				dim.size = 14
				dim.stride = 14
				segment_pos1.layout.dim = [dim]
				segment_pos1.layout.data_offset = 0
				segment_pos1.data = [370.0, 670.0, 650.0, 0.0, 180.0, 0.0, 370.0, 670.0, 650.0, 0.0, 180.0, 0.0, 0.0, 20.0]
				segment_pos.append(segment_pos1)

				""" Segment Pose 2 """
				segment_pos2 = Float64MultiArray()
				# Layout Setting
				segment_pos2.layout.dim = [dim]
				segment_pos2.layout.data_offset = 0
				segment_pos2.data = [370.0, 670.0, 400.0, 0.0, 180.0, 0.0, 370.0, 545.0, 400.0, 0.0, 180.0, 0.0, 1.0, 20.0]
				segment_pos.append(segment_pos2)

				""" Segment Pose 3 """
				segment_pos3 = Float64MultiArray()
				# Layout Setting
				segment_pos3.layout.dim = [dim]
				segment_pos3.layout.data_offset = 0
				segment_pos3.data = [370.0, 670.0, 400.0, 0.0, 180.0, 0.0, 370.0, 670.0, 400.0, 0.0, 180.0, 0.0, 0.0, 20.0]
				segment_pos.append(segment_pos3)

				""" Segment Pose 4 """
				segment_pos4 = Float64MultiArray()
				# Layout Setting
				segment_pos4.layout.dim = [dim]
				segment_pos4.layout.data_offset = 0
				segment_pos4.data = [370.0, 420.0, 150.0, 0.0, 180.0, 0.0, 370.0, 545.0, 150.0, 0.0, 180.0, 0.0, 1.0, 20.0]
				segment_pos.append(segment_pos4)

				""" Segment Pose 5 """
				segment_pos5 = Float64MultiArray()
				# Layout Setting
				segment_pos5.layout.dim = [dim]
				segment_pos5.layout.data_offset = 0
				segment_pos5.data = [370.0, 670.0, 275.0, 0.0, 180.0, 0.0, 370.0, 795.0, 150.0, 0.0, 180.0, 0.0, 1.0, 20.0]
				segment_pos.append(segment_pos5)

				move_blend_req.segment = segment_pos

				move_blend_future = move_blend_cli.call_async(move_blend_req)
				rclpy.spin_until_future_complete(self.node, move_blend_future, timeout_sec=SRV_CALL_TIMEOUT)
				self.assertTrue(move_blend_future.done(), "motion/move_blending service working is not done.")
				move_blend_resp = move_blend_future.result()
				self.assertTrue(move_blend_resp.success == True, "motion/move_blending service is not working correctly.")
				self.node.destroy_client(move_blend_cli)

				if(move_blend_resp.success == True):
					""" Get Current Posj """
					get_current_posj_cli = self.node.create_client(GetCurrentPosj, "aux_control/get_current_posj")
					get_current_posj_future = get_current_posj_cli.call_async(GetCurrentPosj.Request())
					rclpy.spin_until_future_complete(self.node, get_current_posj_future, timeout_sec=SRV_CALL_TIMEOUT)
					self.assertTrue(get_current_posj_future.done(), "aux_control/get_current_posj service working is not done.")
					get_current_posj_resp = get_current_posj_future.result()
					self.assertTrue(get_current_posj_resp.success == True, "aux_control/get_current_posj service is not working correctly.")
					self.node.destroy_client(get_current_posj_cli)

					get_current_posj_result = get_current_posj_resp.pos
					end_posj = [62.813, 43.044, 82.747, 0.000, 54.209, 62.813]
					self.assertAlmostEqual(get_current_posj_result[0], end_posj[0], delta = 1)
					self.assertAlmostEqual(get_current_posj_result[1], end_posj[1], delta = 1)
					self.assertAlmostEqual(get_current_posj_result[2], end_posj[2], delta = 1)
					self.assertAlmostEqual(get_current_posj_result[3], end_posj[3], delta = 1)
					self.assertAlmostEqual(get_current_posj_result[4], end_posj[4], delta = 1)
					self.assertAlmostEqual(get_current_posj_result[5], end_posj[5], delta = 1)

	# Trans Test
	# def test_trans_cli(self):
	# 	print("Move Trans Client Test are starting...") # Debug

	# 	""" Trans """
	# 	trans_cli = self.node.create_client(Trans, "motion/trans")
	# 	trans_req = Trans.Request()
	# 	trans_req.pos = [200.0, 200.0, 200.0, 0.0, 180.0, 0.0]
	# 	trans_req.delta = [100.0, 100.0, 100.0, 0.0, 0.0, 0.0]
	# 	trans_req.ref = 0
	# 	trans_req.ref_out = 0
	# 	trans_future = trans_cli.call_async(trans_req)
	# 	rclpy.spin_until_future_complete(self.node, trans_future, timeout_sec=SRV_CALL_TIMEOUT)
	# 	self.assertTrue(trans_future.done(), "motion/trans service working is not done.")
	# 	trans_resp = trans_future.result()
	# 	self.assertTrue(trans_resp.success == True, "motion/trans service is not working correctly.")
	# 	self.node.destroy_client(trans_cli)

	# 	trans_pos = trans_resp.trans_pos
	# 	target_pos = [300.0, 300.0, 300.0, 45.0, 180.0, 45.0]
	# 	self.assertAlmostEqual(trans_pos[0], target_pos[0], delta=0.01)
	# 	self.assertAlmostEqual(trans_pos[1], target_pos[1], delta=0.01)
	# 	self.assertAlmostEqual(trans_pos[2], target_pos[2], delta=0.01)
	# 	self.assertAlmostEqual(trans_pos[4], target_pos[4], delta=0.01)

	# Jog Test
	def test_jog_cli(self):
		print("Jog Client Test are starting...") # Debug
		""" Jog """
		jog_cli = self.node.create_client(Jog, "motion/jog")
		jog_req = Jog.Request()
		jog_req.jog_axis = 0
		jog_req.move_reference = 0
		jog_req.speed = 60.0
		jog_future = jog_cli.call_async(jog_req)
		rclpy.spin_until_future_complete(self.node, jog_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(jog_future.done(), "motion/jog service working is not done.")
		jog_resp = jog_future.result()
		self.assertTrue(jog_resp.success == True, "motion/jog service is not working correct.")
		self.node.destroy_client(jog_cli)

		""" Motion Stop """
		stop_cli = self.node.create_client(MoveStop, "motion/move_stop")
		stop_req = MoveStop.Request()
		stop_req.stop_mode = 0
		stop_future = stop_cli.call_async(stop_req)
		rclpy.spin_until_future_complete(self.node, stop_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(stop_future.done(), "motion/move_stop future task")
		pause_resp = stop_future.result()
		self.assertTrue(pause_resp.success == True, "motion/move_stop response")
		self.node.destroy_client(stop_future)

""" Motion Setting Service Client Test Class """
class TestDsrSetCli(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		print("=========================================")
		print("===== DooSan Settings Tests Start =======")
		cls._lock = threading.Lock()
		cls.node =rclpy.create_node("test_dsr_set_node", namespace=NAMESPACE)

	@classmethod
	def tearDownClass(cls):
		print("==== DooSan Settings Tests Finished!! ===")
		print("=========================================")

	def setUp(self):
		print("Ready For Settings test!!")
		TestDsrSetCli._lock.acquire()

	def tearDown(self):
		print("Clear For Settings test!!")
		TestDsrSetCli._lock.release()
		
	## Set Reference Coordinate Test
	def test_set_ref_coord_cli(self):
		print("Set Reference Coordinate Client Test are starting...") # Debug
		""" Set Ref Coordinate """
		set_ref_coord_cli = self.node.create_client(SetRefCoord, "motion/set_ref_coord")
		set_ref_coord_req = SetRefCoord.Request()
		set_ref_coord_req.coord = 1 # DR_TOOL
		set_ref_coord_future = set_ref_coord_cli.call_async(set_ref_coord_req)
		rclpy.spin_until_future_complete(self.node, set_ref_coord_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_ref_coord_future.done(), "motion/set_ref_coord service working is not done.")
		set_ref_coord_resp = set_ref_coord_future.result()
		self.assertTrue(set_ref_coord_resp.success == True, "motion/set_ref_coord service is not working correctly.")
		self.node.destroy_client(set_ref_coord_cli)
	# 	""" Need the Code Check to Change Ref Coordinate """

	## Set Singularity Handling Test
	def test_set_singularity_handling_cli(self):
		print("Set Singularity Handling Client Test are starting...") # Debug
		""" Set Singularity Handling """
		singularity_handle_cli = self.node.create_client(SetSingularityHandling, "motion/set_singularity_handling")
		singularity_handle_req = SetSingularityHandling.Request()
		singularity_handle_req.mode = 1
		singularity_handle_future = singularity_handle_cli.call_async(singularity_handle_req)
		rclpy.spin_until_future_complete(self.node, singularity_handle_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(singularity_handle_future.done(), "motion/set_singularity_handling service working is not done.")
		singularity_handle_resp = singularity_handle_future.result()
		self.assertTrue(singularity_handle_resp.success == True, "motion/set_singularity_handling service is not working correctly.")
		self.node.destroy_client(singularity_handle_cli)

	# 	""" Need to Code Check to Change Singularity Handling Mode """

	## Change Operation Speed Test
	def test_change_operation_speed_cli(self):
		print("Change Operation Speed Client Test are starting...") # Debug
		""" Change Operation Speed """
		# 10% speed
		change_operation_speed_cli = self.node.create_client(ChangeOperationSpeed, "motion/change_operation_speed")
		change_operation_speed_req = ChangeOperationSpeed.Request()
		change_operation_speed_req.speed = 10
		change_operation_speed_future = change_operation_speed_cli.call_async(change_operation_speed_req)
		rclpy.spin_until_future_complete(self.node, change_operation_speed_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(change_operation_speed_future.done(), "motion/change_operation_speed service working is not done.")
		change_operation_speed_resp = change_operation_speed_future.result()
		self.assertTrue(change_operation_speed_resp.success == True, "motion/change_operation_speed service is not working correctly.")

		""" Need the code to check changed operation speed """


	## Enable Alter Motion Test
	def test_enable_alter_motion_cli(self):
		print("Enable Alter Motion Client Test are starting...") # Debug
		""" Enable Alter Motion """
		enable_alter_motion_cli = self.node.create_client(EnableAlterMotion, "motion/enable_alter_motion")
		enable_alter_motion_req = EnableAlterMotion.Request()
		enable_alter_motion_req.n = 5
		enable_alter_motion_req.mode = 0 # DR_DPOS : accumulation amount
		enable_alter_motion_req.ref = 0 # DR_BASE
		enable_alter_motion_req.limit_dpos = [50.0, 90.0]
		enable_alter_motion_req.limit_dpos_per = [10.0, 10.0]
		enable_alter_motion_future = enable_alter_motion_cli.call_async(enable_alter_motion_req)
		rclpy.spin_until_future_complete(self.node, enable_alter_motion_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(enable_alter_motion_future.done(), "motion/enable_alter_motion service working is not doen.")
		enable_alter_motion_resp = enable_alter_motion_future.result()
		self.assertTrue(enable_alter_motion_resp.success == True, "motion/enable_alter_motion service is not working correctly.")
		self.node.destroy_client(enable_alter_motion_cli)


	## Disalbe Alter Motion Test
	def test_disable_alter_motion_cli(self):
		print("Disable Alter Motion Client Test are starting...") # Debug
		""" Disable Alter Motion """
		disable_alter_motion_cli = self.node.create_client(DisableAlterMotion, "motion/disable_alter_motion")
		disable_alter_motion_future = disable_alter_motion_cli.call_async(DisableAlterMotion.Request())
		rclpy.spin_until_future_complete(self.node, disable_alter_motion_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(disable_alter_motion_future.done(), "motion/disable_alter_motion service working is not done.")
		disable_alter_motion_resp = disable_alter_motion_future.result()
		self.assertTrue(disable_alter_motion_resp.success == True, "motion/disable_alter_motion service is not working correctly.")
		self.node.destroy_client(disable_alter_motion_cli)

	""" TODO : Need the code check alter motion real operation """
	## Alter Motion Test
	def test_alter_motion_cli(self):
		print("Alter Motion Client Test are starting...") # Debug
		alter_motion_cli = self.node.create_client(AlterMotion, "motion/alter_motion")
		alter_motion_req = AlterMotion.Request()
		alter_motion_req.pos = [10.0, 0.0, 0.0, 10.0, 0.0, 0.0]
		alter_motion_future = alter_motion_cli.call_async(alter_motion_req)
		rclpy.spin_until_future_complete(self.node, alter_motion_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(alter_motion_future.done(), "motion/alter_motion service working is not done.")
		alter_motion_resp = alter_motion_future.result()
		self.assertTrue(alter_motion_resp.success == True, "motion/alter_motion service is not working correctly.")
		self.node.destroy_client(alter_motion_cli)


	## FKin Test
	def test_fkin_cli(self):
		print("Fkin Client Test are starting...") # Debug
		""" Fkin """
		fkin_cli = self.node.create_client(Fkin, "motion/fkin")
		fkin_req = Fkin.Request()
		fkin_req.pos = [30.0, 0.0, 90.0, 0.0, 90.0, 0.0]
		fkin_req.ref = 2
		fkin_future = fkin_cli.call_async(fkin_req)
		rclpy.spin_until_future_complete(self.node, fkin_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(fkin_future.done(), "motion/fkin service working is not done.")
		fkin_resp = fkin_future.result()
		self.assertTrue(fkin_resp.success == True, "motion/fkin service is not working correctly.")

		# Check Result
		fkin_result_pos = fkin_resp.conv_posx
		target_pos = [466.9, 309.4, 651.5, 75.0, 180.0, 45.0]
		self.assertAlmostEqual(target_pos[0], fkin_result_pos[0], delta=0.1)
		self.assertAlmostEqual(target_pos[1], fkin_result_pos[1], delta=0.1)
		self.assertAlmostEqual(target_pos[2], fkin_result_pos[2], delta=0.1)
		self.assertAlmostEqual(target_pos[3], fkin_result_pos[3], delta=0.1)
		self.assertAlmostEqual(target_pos[4], fkin_result_pos[4], delta=0.1)
		self.assertAlmostEqual(target_pos[5], fkin_result_pos[5], delta=0.1)
		self.node.destroy_client(fkin_cli)

	## Ikin Test
	def test_ikin_cli(self):
		print("Ikin Client Test are starting...") # Debug
		""" Ikin """
		ikin_cli = self.node.create_client(Ikin, "motion/ikin")
		ikin_req = Ikin.Request()
		ikin_req.pos = [370.9, 719.7, 651.5, 90.0, -180.0, 0.0]
		ikin_req.sol_space = 2
		ikin_req.ref = 0
		ikin_future = ikin_cli.call_async(ikin_req)
		rclpy.spin_until_future_complete(self.node, ikin_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(ikin_future.done(), "motion/ikin service working is not done.")
		ikin_resp = ikin_future.result()
		self.assertTrue(ikin_resp.success == True, "moiton/ikin service is not working correctly.")

		# Check Result
		ikin_result_pos = ikin_resp.conv_posj
		target_pos = [60.3, 24.0, 60.5, 0.0, 95.5, -29.7]
		self.assertAlmostEqual(target_pos[0], ikin_result_pos[0], delta=0.1)
		self.assertAlmostEqual(target_pos[1], ikin_result_pos[1], delta=0.1)
		self.assertAlmostEqual(target_pos[2], ikin_result_pos[2], delta=0.1)
		self.assertAlmostEqual(target_pos[3], ikin_result_pos[3], delta=0.1)
		self.assertAlmostEqual(target_pos[4], ikin_result_pos[4], delta=0.1)
		self.assertAlmostEqual(target_pos[5], ikin_result_pos[5], delta=0.1)
		self.node.destroy_client(ikin_cli)


""" System Service Client Test Class """
@pytest.mark.usefixtures("bringUp")
class TestDsrSysCli(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		print("====================================================")
		print("===== DooSanSystem System Settings Tests Start =======")
		cls.node =rclpy.create_node("test_dsr_sys_node", namespace=NAMESPACE)
		cls._lock = threading.Lock()

	@classmethod
	def tearDownClass(cls):
		print("===== DooSanSystem System Settings Tests Ends =======")
		print("====================================================")

	def setUp(self):
		print("Ready For System Settings Test!!")
		TestDsrSysCli._lock.acquire()

	def tearDown(self):
		print("Clear For System Settings Test!!")
		TestDsrSysCli._lock.release()

	""" System Client """
	## Get Robot State Test 
	def test_get_robot_state_cli(self):
		print("Get Robot State Client Test are starting...") # Debug

		""" Move Joint """
		move_cli = self.node.create_client(MoveJoint, "motion/move_joint")
		target_pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
		move_req = MoveJoint.Request()
		move_req.pos = target_pos
		move_req.time = 5.0
		move_req.sync_type = 1
		move_future = move_cli.call_async(move_req)
		rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(move_future.done(), "motion/move joint future task")
		move_resp = move_future.result()
		self.assertTrue(move_resp.success == True, "motion/move joint response")
		self.node.destroy_client(move_cli)

		""" Get Robot State """
		get_state_cli = self.node.create_client(GetRobotState, "system/get_robot_state")
		get_state_req = GetRobotState.Request()
		get_state_future = get_state_cli.call_async(get_state_req)
		rclpy.spin_until_future_complete(self.node, get_state_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_state_future.done(), "system/get_robot_mode future task")
		get_state_resp = get_state_future.result()
		# Receiving a command and completing the action, automatically switch to STATE_STANDBY
		self.assertTrue((get_state_resp.robot_state == 1) and (get_state_resp.success == True), "system/get_robot_mode response")
		self.node.destroy_client(get_state_cli)


	""" TODO : Need Code Modification to allow checking the transition and changes in operational state """
	## Set Robot Control(State) Test
	def test_set_robot_control_cli(self):
		print("Set Robot Control Client Test are starting...") # Debug
		""" Set Robot Control(State) """
		set_control_cli = self.node.create_client(SetRobotControl, "system/set_robot_control")
		set_control_req = SetRobotControl.Request(robot_control=0)
		set_control_future = set_control_cli.call_async(set_control_req)
		rclpy.spin_until_future_complete(self.node, set_control_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_control_future.done(), "system/set_robot_control future task is not done.")
		set_control_resp = set_control_future.result()
		self.assertTrue(set_control_resp.success == True, "system/set_robot_control is not working.")
		self.node.destroy_client(set_control_cli)
	

	### Get Robot Mode Test 
	def test_get_robot_mode_cli(self):
		print("Get Robot Mode Client Test are starting...") # Debug
		""" Get Robot Mode """
		get_mode_cli = self.node.create_client(GetRobotMode, "system/get_robot_mode")
		get_mode_req = GetRobotMode.Request()
		get_mode_future = get_mode_cli.call_async(get_mode_req)
		rclpy.spin_until_future_complete(self.node, get_mode_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_mode_future.done(), "system/get_robot_mode future task done")
		get_mode_resp = get_mode_future.result()
		self.assertTrue((get_mode_resp.robot_mode == 1) and (get_mode_resp.success == True), "system/get_robot_mode response")
		self.node.destroy_client(get_mode_cli)


	## Set Robot Mode Test
	def test_set_robot_mode_cli(self):
		print("Set Robot Mode Client Test are starting...") # Debug
		""" Set Robot Mode """
		set_mode_cli = self.node.create_client(SetRobotMode, "system/set_robot_mode")
		set_mode_req = SetRobotMode.Request()
		set_mode_req.robot_mode = 1
		set_mode_future = set_mode_cli.call_async(set_mode_req)
		rclpy.spin_until_future_complete(self.node, set_mode_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_mode_future.done(), "system/set_robot_mode Service is not done.")
		set_mode_resp = set_mode_future.result()
		self.assertTrue(set_mode_resp.success == True, "system/set_robot_mode Service is not working.")
		self.node.destroy_client(set_mode_cli)

		if(set_mode_resp.success == True):
			""" Get Robot Mode """
			get_mode_cli = self.node.create_client(GetRobotMode, "system/get_robot_mode")
			get_mode_req = GetRobotMode.Request()
			get_mode_future = get_mode_cli.call_async(get_mode_req)
			rclpy.spin_until_future_complete(self.node, get_mode_future, timeout_sec=SRV_CALL_TIMEOUT)
			self.assertTrue(get_mode_future.done(), "system/get_robot_mode future task done")
			get_mode_resp = get_mode_future.result()
			self.assertTrue((get_mode_resp.robot_mode == 1) and (get_mode_resp.success == True), "system/get_robot_mode response")
			self.node.destroy_client(get_mode_cli)


	### Get Robot Speed Mode Test
	def test_get_robot_speed_mode_cli(self):
		print("Get Robot Speed Mode Client Test are starting...") # Debug
		""" Get Robot Speed Mode """
		get_speed_mode_cli = self.node.create_client(GetRobotSpeedMode, "system/get_robot_speed_mode")
		get_speed_mode_req = GetRobotSpeedMode.Request()
		get_speed_mode_future = get_speed_mode_cli.call_async(get_speed_mode_req)
		rclpy.spin_until_future_complete(self.node, get_speed_mode_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_speed_mode_future.done(), "system/get_robot_speed_mode future task is not done")
		get_speed_mode_resp = get_speed_mode_future.result()
		self.assertTrue((get_speed_mode_resp.speed_mode == 0) and (get_speed_mode_resp.success == True), "system/get_robot_speed_mode result is not correct")
		self.node.destroy_client(get_speed_mode_cli)


	## Set Robot Speed Mode Test
	def test_set_robot_speed_mode_cli(self):
		print("Set Robot Speed Mode Client Test are starting...") # Debug
		""" Set Robot Speed Mode """
		set_speed_mode_cli = self.node.create_client(SetRobotSpeedMode, "system/set_robot_speed_mode")
		set_speed_mode_req = SetRobotSpeedMode.Request()
		set_speed_mode_req.speed_mode = 1
		set_speed_mode_future = set_speed_mode_cli.call_async(set_speed_mode_req)
		rclpy.spin_until_future_complete(self.node, set_speed_mode_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_speed_mode_future.done(), "system/set_robot_speed_mode future task is not done.")
		set_speed_mode_resp = set_speed_mode_future.result()
		self.assertTrue(set_speed_mode_resp.success == True, "system/set_robot_speed_mode Service is not working correctly.")

		if(set_speed_mode_resp.success == True):
			get_speed_mode_cli = self.node.create_client(GetRobotSpeedMode, "system/get_robot_speed_mode")
			get_speed_mode_req = GetRobotSpeedMode.Request()
			get_speed_mode_future = get_speed_mode_cli.call_async(get_speed_mode_req)
			rclpy.spin_until_future_complete(self.node, get_speed_mode_future, timeout_sec=SRV_CALL_TIMEOUT)
			self.assertTrue(get_speed_mode_future.done(), "system/get_robot_speed_mode future task is not done")
			get_speed_mode_resp = get_speed_mode_future.result()
			self.assertTrue((get_speed_mode_resp.speed_mode == 1) and (get_speed_mode_resp.success == True), "system/get_robot_speed_mode result is not correct")
			self.node.destroy_client(get_speed_mode_cli)


	## Get Robot System Test
	def test_get_robot_system_cli(self):
		print("Get Robot System Client Test are starting...") # Debug
		""" Get Robot System """
		get_system_cli = self.node.create_client(GetRobotSystem, "system/get_robot_system")
		get_system_req = GetRobotSystem.Request()
		get_system_future = get_system_cli.call_async(get_system_req)
		rclpy.spin_until_future_complete(self.node, get_system_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_system_future.done(), "system/get_robot_system future task is not done.")
		get_system_resp = get_system_future.result()
		self.assertTrue((get_system_resp.robot_system == 1) and (get_system_resp.success == True), "system/get_robot_system result is not correct.")
		self.node.destroy_client(get_system_cli)


	## Set Robot System Test
	def test_set_robot_system_cli(self):
		print("Set Robot System Client Test are starting...") # Debug
		""" Set Robot System """
		set_system_cli = self.node.create_client(SetRobotSystem, "system/set_robot_system")
		set_system_req = SetRobotSystem.Request()
		set_system_req.robot_system = 0
		set_system_future = set_system_cli.call_async(set_system_req)
		rclpy.spin_until_future_complete(self.node, set_system_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_system_future.done(), "system/set_robot_system Service is not done.")
		set_system_resp = set_system_future.result()
		self.assertTrue(set_system_resp.success == True, "system/set_robot_system Service is not working corretly.")

		if(set_system_resp.success == True):
			get_system_cli = self.node.create_client(GetRobotSystem, "system/get_robot_system")
			get_system_req = GetRobotSystem.Request()
			get_system_future = get_system_cli.call_async(get_system_req)
			rclpy.spin_until_future_complete(self.node, get_system_future, timeout_sec=SRV_CALL_TIMEOUT)
			self.assertTrue(get_system_future.done(), "system/get_robot_system future task is not done.")
			get_system_resp = get_system_future.result()
			self.assertTrue((get_system_resp.robot_system == 0) and (get_system_resp.success == True), "system/get_robot_system result is not correct.")
			self.node.destroy_client(get_system_cli)


	## ServoOff Test
	def test_servo_off_cli(self):
		print("Servo Off Client Test are starting...") # Debug
		""" Servo Off """
		servo_off_cli = self.node.create_client(ServoOff, "system/servo_off")
		servo_off_req = ServoOff.Request()
		servo_off_req.stop_type = 0
		servo_off_future = servo_off_cli.call_async(servo_off_req)
		rclpy.spin_until_future_complete(self.node, servo_off_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(servo_off_future.done(), "system/servo_off future task is not done.")
		servo_off_resp = servo_off_future.result()
		self.assertTrue(servo_off_resp.success == True, "system/servo_off Service is not working.")
		self.node.destroy_client(servo_off_cli)


	## SetSafeStopResetType Test
	def test_set_safe_stop_reset_type_cli(self):
		print("Set Safe Stop Reset Type Client Test are starting...") # Debug
		set_safestopreset_cli = self.node.create_client(SetSafeStopResetType, "system/set_safe_stop_reset_type")
		set_safestopreset_req = SetSafeStopResetType.Request()
		set_safestopreset_req.reset_type = 1
		set_safestopreset_future = set_safestopreset_cli.call_async(set_safestopreset_req)
		rclpy.spin_until_future_complete(self.node, set_safestopreset_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_safestopreset_future.done(), "system/set_safe_stop_reset_type service is not done.")
		set_safestopreset_resp = set_safestopreset_future.result()
		self.assertTrue(set_safestopreset_resp.success == True, "system/set_safe_stop_reset_type service is not working correctly.")
		self.node.destroy_client(set_safestopreset_cli)


	## Change Collision Sensitivity Test
	def test_change_collision_sensitivity_cli(self):
		print("Change Collision Sensitivity Client Test are starting...") # Debug
		change_collision_sense_cli = self.node.create_client(ChangeCollisionSensitivity, "system/change_collision_sensitivity")
		change_collision_sense_req = ChangeCollisionSensitivity.Request()
		change_collision_sense_req.sensitivity = 50
		change_collision_sense_future = change_collision_sense_cli.call_async(change_collision_sense_req)
		rclpy.spin_until_future_complete(self.node, change_collision_sense_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(change_collision_sense_future.done(), "system/change_collision_sensitivity service is not done.")
		change_collision_sense_resp = change_collision_sense_future.result()
		self.assertTrue(change_collision_sense_resp.success == True, "system/change_collision_sensitivity service is not working correctly.")
		self.node.destroy_client(change_collision_sense_cli)


	## Get Last Alarm Test
	def test_get_last_alarm_cli(self):
		print("Get Last Alarm Client Test are starting...") # Debug
		get_last_alarm_cli = self.node.create_client(GetLastAlarm, "system/get_last_alarm")
		get_last_alarm_req = GetLastAlarm.Request()
		get_last_alarm_future = get_last_alarm_cli.call_async(get_last_alarm_req)
		rclpy.spin_until_future_complete(self.node, get_last_alarm_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_last_alarm_future.done(), "system/get_last_alarm is not done.")
		get_last_alarm_resp = get_last_alarm_future.result()
		self.assertTrue(get_last_alarm_resp.success == True, "system/get_last_alarm is not working correctly.")
		self.node.destroy_client(get_last_alarm_cli)


""" Aux Control Service Client Test Class """
class TestDsrAuxCtrlCli(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		print("====================================================")
		print("===== DooSanSystem Aux Ctrl Tests Start =======")
		cls._lock = threading.Lock()
		cls.node =rclpy.create_node("dsr_aux_control_test_node", namespace=NAMESPACE)

	@classmethod
	def tearDownClass(cls):
		print("===== DooSanSystem Aux Ctrl Tests Ends =======")
		print("====================================================")
	
	def setUp(self):
		print("Ready For Aux Ctrl Tests!!")
		TestDsrAuxCtrlCli._lock.acquire()

	def tearDown(self):
		print("Clear For Aux Ctrl Tests!!")
		TestDsrAuxCtrlCli._lock.release()

	# Get Control Mode Test
	def test_get_control_mode_cli(self):
		print("Get Control Mode Client Test are starting...") # Debug
		""" Get Control Mode """
		get_control_mode_cli = self.node.create_client(GetControlMode, "aux_control/get_control_mode")
		get_control_mode_future = get_control_mode_cli.call_async(GetControlMode.Request())
		rclpy.spin_until_future_complete(self.node, get_control_mode_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_control_mode_future.done(), "aux_control/get_control_mode service working is not done.")
		get_control_mode_resp = get_control_mode_future.result()
		print(get_control_mode_resp)
		self.assertTrue((get_control_mode_resp.success == True), "aux_control/get_control_mode service is not working correctly.")
		self.node.destroy_client(get_control_mode_cli)


	# Get Control Space Test
	def test_get_control_space_cli(self):
		print("Get Control Space Client Test are starting...") # Debug
		""" Get Control Space """
		get_control_space_cli = self.node.create_client(GetControlSpace, "aux_control/get_control_space")
		get_control_space_future = get_control_space_cli.call_async(GetControlSpace.Request())
		rclpy.spin_until_future_complete(self.node, get_control_space_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_control_space_future.done(), "aux_control/get_control_space service working is not done.")
		get_control_space_resp = get_control_space_future.result()
		print(get_control_space_resp)
		self.assertTrue((get_control_space_resp.success == True), "aux_control/get_control_space service is not working correctly.")
		self.node.destroy_client(get_control_space_cli)


	# Get Current Rotm Test
	def test_get_current_rotm_cli(self):
		print("Get Current Rotation Matrix Client Test are starting...") # Debug
		""" Get Current Rotm """
		get_current_rotm_cli = self.node.create_client(GetCurrentRotm, "aux_control/get_current_rotm")
		get_current_rotm_future = get_current_rotm_cli.call_async(GetCurrentRotm.Request(ref=0))
		rclpy.spin_until_future_complete(self.node, get_current_rotm_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_current_rotm_future.done(), "aux_control/get_current_rotm service working is not done.")
		get_current_rotm_resp = get_current_rotm_future.result()
		print(get_current_rotm_resp)
		self.assertTrue(get_current_rotm_resp.success == True, "aux_control/get_current_rotm service is not working correctly.")
		self.node.destroy_client(get_current_rotm_cli)


	# Get Current Solution Space Test
	def test_get_current_solution_space_cli(self):
		print("Get Current Solution Space Client Test are starting...") # Debug
		""" Get Current Rotm """
		get_current_solution_space_cli = self.node.create_client(GetCurrentSolutionSpace, "aux_control/get_current_solution_space")
		get_current_solution_space_future = get_current_solution_space_cli.call_async(GetCurrentSolutionSpace.Request())
		rclpy.spin_until_future_complete(self.node, get_current_solution_space_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_current_solution_space_future.done(), "aux_control/get_current_solution_space service working is not done.")
		get_current_solution_space_resp = get_current_solution_space_future.result()
		print(get_current_solution_space_resp)
		self.assertTrue(get_current_solution_space_resp.success == True, "aux_control/get_current_solution_space service is not working correctly.")
		self.node.destroy_client(get_current_solution_space_cli)


	# Get Current Tool Flange Posx Test
	def test_get_current_tool_flange_posx_cli(self):
		print("Get Current Tool Flange Posx Client Test are starting...") # Debug
		""" Get Current Tool Flange Posx """
		get_current_tool_flange_posx_cli = self.node.create_client(GetCurrentToolFlangePosx, "aux_control/get_current_tool_flange_posx")
		get_current_tool_flange_posx_future = get_current_tool_flange_posx_cli.call_async(GetCurrentToolFlangePosx.Request(ref=0))
		rclpy.spin_until_future_complete(self.node, get_current_tool_flange_posx_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_current_tool_flange_posx_future.done(), "aux_control/get_current_tool_flange_posx service working is not done.")
		get_current_tool_flange_posx_resp = get_current_tool_flange_posx_future.result()
		print(get_current_tool_flange_posx_resp)
		self.assertTrue(get_current_tool_flange_posx_resp.success == True, "aux_control/get_current_tool_flange_posx service is not working correctly.")
		self.node.destroy_client(get_current_tool_flange_posx_cli)


	# Get Current Velj Test
	def test_get_current_velj_cli(self):
		print("Get Current Joint Velocigy Client Test are starting...") # Debug
		""" Get Current Velj """
		get_current_velj_cli = self.node.create_client(GetCurrentVelj, "aux_control/get_current_velj")
		get_current_velj_future = get_current_velj_cli.call_async(GetCurrentVelj.Request())
		rclpy.spin_until_future_complete(self.node, get_current_velj_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_current_velj_future.done(), "aux_control/get_current_velj service working is not done.")
		get_current_velj_resp = get_current_velj_future.result()
		print(get_current_velj_resp)
		self.assertTrue(get_current_velj_resp.success == True, "aux_control/get_current_velj service is not working correctly.")
		self.node.destroy_client(get_current_velj_cli)


	# Get Current Velx Test
	def test_get_current_velx_cli(self):
		print("Get Current Task Space Velocigy Client Test are starting...") # Debug
		""" Get Current Velx """
		get_current_velx_cli = self.node.create_client(GetCurrentVelx, "aux_control/get_current_velx")
		get_current_velx_future = get_current_velx_cli.call_async(GetCurrentVelx.Request(ref=0))
		rclpy.spin_until_future_complete(self.node, get_current_velx_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_current_velx_future.done(), "aux_control/get_current_velx service working is not done.")
		get_current_velx_resp = get_current_velx_future.result()
		print(get_current_velx_resp)
		self.assertTrue(get_current_velx_resp.success == True, "aux_control/get_current_velx service is not working correctly.")
		self.node.destroy_client(get_current_velx_cli)


	# Get Desired Posj Test
	def test_get_desired_posj_cli(self):
		print("Get Desired Joint Position Client Test are starting...") # Debug
		""" Get Desired Posj """
		get_desired_posj_cli = self.node.create_client(GetDesiredPosj, "aux_control/get_desired_posj")
		get_desired_posj_future = get_desired_posj_cli.call_async(GetDesiredPosj.Request())
		rclpy.spin_until_future_complete(self.node, get_desired_posj_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_desired_posj_future.done(), "aux_control/get_desired_posj service working is not done.")
		get_desired_posj_resp = get_desired_posj_future.result()
		print(get_desired_posj_resp)
		self.assertTrue(get_desired_posj_resp.success == True, "aux_control/get_desired_posj service is not working correctly.")
		self.node.destroy_client(get_desired_posj_cli) 


	# Get Desired Posx Test
	def test_get_desired_posx_cli(self):
		print("Get Desired Task Space Position Client Test are starting...") # Debug
		""" Get Desired Posx """
		get_desired_posx_cli = self.node.create_client(GetDesiredPosx, "aux_control/get_desired_posx")
		get_desired_posx_future = get_desired_posx_cli.call_async(GetDesiredPosx.Request(ref=0))
		rclpy.spin_until_future_complete(self.node, get_desired_posx_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_desired_posx_future.done(), "aux_control/get_desired_posx service working is not done.")
		get_desired_posx_resp = get_desired_posx_future.result()
		print(get_desired_posx_resp)
		self.assertTrue(get_desired_posx_resp.success == True, "aux_control/get_desired_posx service is not working correctly.")
		self.node.destroy_client(get_desired_posx_cli)


	# Get Desired Velj Test
	def test_get_desired_velj_cli(self):
		print("Get Desired Joint Velocity Client Test are starting...") # Debug
		""" Get Desired Velj """
		get_desired_velj_cli = self.node.create_client(GetDesiredVelj, "aux_control/get_desired_velj")
		get_desired_velj_future = get_desired_velj_cli.call_async(GetDesiredVelj.Request())
		rclpy.spin_until_future_complete(self.node, get_desired_velj_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_desired_velj_future.done(), "aux_control/get_desired_velj service working is not done.")
		get_desired_velj_resp = get_desired_velj_future.result()
		print(get_desired_velj_resp)
		self.assertTrue(get_desired_velj_resp.success == True, "aux_control/get_desired_velj service is not working correctly.")
		self.node.destroy_client(get_desired_velj_cli)

	
	# Get Desired Velx Test
	def test_get_desired_velx_cli(self):
		print("Get Desired Task Space Velocity Client Test are starting...") # Debug
		""" Get Desired Velx """
		get_desired_velx_cli = self.node.create_client(GetDesiredVelx, "aux_control/get_desired_velx")
		get_desired_velx_future = get_desired_velx_cli.call_async(GetDesiredVelx.Request(ref=0))
		rclpy.spin_until_future_complete(self.node, get_desired_velx_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_desired_velx_future.done(), "aux_control/get_desired_velx service working is not done.")
		get_desired_velx_resp = get_desired_velx_future.result()
		print(get_desired_velx_resp)
		self.assertTrue(get_desired_velx_resp.success == True, "aux_control/get_desired_velx service is not working correctly.")
		self.node.destroy_client(get_desired_velx_cli)

	
	# Get External Torque Test
	def test_get_external_torque_cli(self):
		print("Get External Torque Client Test are starting...") # Debug
		""" Get External Torque """
		get_external_torque_cli = self.node.create_client(GetExternalTorque, "aux_control/get_external_torque")
		get_external_torque_future = get_external_torque_cli.call_async(GetExternalTorque.Request())
		rclpy.spin_until_future_complete(self.node, get_external_torque_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_external_torque_future.done(), "aux_control/get_external_torque service working is not done.")
		get_external_torque_resp = get_external_torque_future.result()
		print(get_external_torque_resp)
		self.assertTrue(get_external_torque_resp.success == True, "aux_control/get_external_torque service is not working correctly.")
		self.node.destroy_client(get_external_torque_cli)


	# Get Orientaion Error Test
	def test_get_orientation_error_cli(self):
		print("Get Orientation Error Client Test are starting...") # Debug
		""" Get Orientaion Error """
		get_orientation_error_cli = self.node.create_client(GetOrientationError, "aux_control/get_orientation_error")
		get_orientation_error_req = GetOrientationError.Request()
		get_orientation_error_req.xd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		get_orientation_error_req.xc = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
		get_orientation_error_req.axis = 0
		get_orientation_error_future = get_orientation_error_cli.call_async(get_orientation_error_req)
		rclpy.spin_until_future_complete(self.node, get_orientation_error_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_orientation_error_future.done(), "aux_control/get_orientation_error service working is not done.")
		get_orientation_error_resp = get_orientation_error_future.result()
		print(get_orientation_error_resp)
		self.assertTrue(get_orientation_error_resp.success == True, "aux_control/get_orientation_error service is not working correctly.")
		self.node.destroy_client(get_orientation_error_cli)


	# Get Solution Space Test
	def test_get_solution_space_cli(self):
		print("Get Solution Space Client Test are starting...") # Debug
		""" Get Solution Space """
		get_solution_space_cli = self.node.create_client(GetSolutionSpace, "aux_control/get_solution_space")
		get_solution_space_req = GetSolutionSpace.Request()
		get_solution_space_req.pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		get_solution_space_future = get_solution_space_cli.call_async(get_solution_space_req)
		rclpy.spin_until_future_complete(self.node, get_solution_space_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_solution_space_future.done(), "aux_control/get_solution_space service working is not done.")
		get_solution_space_resp = get_solution_space_future.result()
		print(get_solution_space_resp)
		self.assertTrue(get_solution_space_resp.success == True, "aux_control/get_solution_space service is not working correctly.")
		self.node.destroy_client(get_solution_space_cli)


	# Get Tool Force Test
	def test_get_tool_force_cli(self):
		print("Get Tool Force Client Test are starting...") # Debug

		""" Get Tool Force """
		get_tool_force_cli = self.node.create_client(GetToolForce, "aux_control/get_tool_force")
		get_tool_force_req = GetToolForce.Request()
		get_tool_force_req.ref = 0
		get_tool_force_future = get_tool_force_cli.call_async(get_tool_force_req)
		rclpy.spin_until_future_complete(self.node, get_tool_force_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_tool_force_future.done(), "aux_control/get_tool_force service working is not done.")
		get_tool_force_resp = get_tool_force_future.result()
		print(get_tool_force_resp)
		self.assertTrue(get_tool_force_resp.success == True, "aux_control/get_tool_force service is not working correctly.")
		self.node.destroy_client(get_tool_force_cli)


""" Tool Service Client Test Class """
class TestDsrToolCli(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		print("====================================================")
		print("===== DooSanSystem Tool Tests Start ================")
		cls._lock = threading.Lock()
		cls.node =rclpy.create_node("dsr_tool_test_node", namespace=NAMESPACE)

	@classmethod
	def tearDownClass(cls):
		print("===== DooSanSystem Tool Tests Ends ================")
		print("====================================================")

	def setUp(self):
		print("Ready For Tool Test!!")
		TestDsrToolCli._lock.acquire()

	def tearDown(self):
		print("Clear For Tool Test!!")
		TestDsrToolCli._lock.release()

	# Get Current Tool Test
	def test_get_current_tool_cli(self):
		print("Get Current Tool Client Test are starting...") # Debug
  
		""" Set Robot Mode """
		set_mode_cli = self.node.create_client(SetRobotMode, "system/set_robot_mode")
		set_mode_req = SetRobotMode.Request()
		set_mode_req.robot_mode = 0
		set_mode_future = set_mode_cli.call_async(set_mode_req)
		rclpy.spin_until_future_complete(self.node, set_mode_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_mode_future.done(), "system/set_robot_mode Service is not done.")
		set_mode_resp = set_mode_future.result()
		self.assertTrue(set_mode_resp.success == True, "system/set_robot_mode Service is not working.")
  
		""" Get Current Tool """
		get_current_tool_cli = self.node.create_client(GetCurrentTool, "tool/get_current_tool")
		get_current_tool_future = get_current_tool_cli.call_async(GetCurrentTool.Request())
		rclpy.spin_until_future_complete(self.node, get_current_tool_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_current_tool_future.done(), "tool/get_current_tool service working is not done.")
		get_current_tool_resp = get_current_tool_future.result()
		self.assertTrue((get_current_tool_resp.success == True) and (get_current_tool_resp.info == ""), "tool/get_current_tool service is not working correctly.")
		self.node.destroy_client(get_current_tool_cli)


	# Set Tool Shape Test
	def test_set_tool_shape_cli(self):
		print("Set Tool Shape Client Test are starting...") # Debug

		""" Set Tool Shape """
		set_tool_shape_cli = self.node.create_client(SetToolShape, "tool/set_tool_shape")
		set_tool_shape_future = set_tool_shape_cli.call_async(SetToolShape.Request(name="tool_shape1"))
		rclpy.spin_until_future_complete(self.node, set_tool_shape_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_tool_shape_future.done(), "tool/set_tool_shape service working is not done.")
		set_tool_shape_resp = set_tool_shape_future.result()
		self.assertTrue(set_tool_shape_resp.success == True, "tool/set_tool_shape service is not working correctly.")
		self.node.destroy_client(set_tool_shape_cli)

	# # Config Create Tool  Test
	# def test_config_create_tool_cli(self):
	# 	print("Configuration Create Tool Client Test are starting...") # Debug

	# 	""" Create Tool """
	# 	config_create_tool_cli = self.node.create_client(ConfigCreateTool, "tool/config_create_tool")
	# 	config_create_tool_req = ConfigCreateTool.Request()
	# 	config_create_tool_req.name = "tool1"
	# 	config_create_tool_req.weight = 5.3
	# 	config_create_tool_req.cog = [10.0, 10.0, 10.0]
	# 	config_create_tool_req.inertia = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	# 	config_create_tool_future = config_create_tool_cli.call_async(config_create_tool_req)
	# 	rclpy.spin_until_future_complete(self.node, config_create_tool_future, timeout_sec=SRV_CALL_TIMEOUT)
	# 	self.assertTrue(config_create_tool_future.done(), "tool/config_create_tool service working is not done.")
	# 	config_create_tool_resp = config_create_tool_future.result()
	# 	self.assertTrue(config_create_tool_resp.success == True, "tool/config_create_tool service is not working correctly.")
	# 	self.node.destroy_client(config_create_tool_resp)

	# # Set Current Tool  Test
	# def test_set_current_tool_cli(self):
	# 	print("Set Current Tool Client Test are starting...") # Debug

	# 	""" Set Current Tool """
	# 	set_current_tool_cli = self.node.create_client(SetCurrentTool, "tool/set_current_tool")
	# 	set_current_tool_future = set_current_tool_cli.call_async(SetCurrentTool.Request(name="tool1"))
	# 	rclpy.spin_until_future_complete(self.node, set_current_tool_future, timeout_sec=SRV_CALL_TIMEOUT)
	# 	self.assertTrue(set_current_tool_future.done(), "tool/set_current_tool service working is not done.")
	# 	set_current_tool_resp = set_current_tool_future.result()
	# 	self.assertTrue(set_current_tool_resp.success == True, "tool/set_current_tool service is not working correctly.")
	# 	self.node.destroy_client(set_current_tool_cli)

	# 	time.sleep(5) ### Note: we added sleep to make sure set_current_tool works (TODO: Fix service)

	# 	""" Get Current Tool """
	# 	get_current_tool_cli = self.node.create_client(GetCurrentTool, "tool/get_current_tool")
	# 	get_current_tool_future = get_current_tool_cli.call_async(GetCurrentTool.Request())
	# 	rclpy.spin_until_future_complete(self.node, get_current_tool_future, timeout_sec=SRV_CALL_TIMEOUT)
	# 	self.assertTrue(get_current_tool_future.done(), "tool/get_current_tool service working is not done.")
	# 	get_current_tool_resp = get_current_tool_future.result()
	# 	print(get_current_tool_resp)
	# 	self.assertTrue((get_current_tool_resp.success == True) and (get_current_tool_resp.info == "tool1"), "tool/get_current_tool service is not working correctly.")
	# 	self.node.destroy_client(get_current_tool_cli)

	# # Config Delete Tool Test
	# def test_config_delete_tool_cli(self):
	# 	print("Configuration Delete Tool Client Test are starting...") # Debug

	# 	""" Config Create Tool """
	# 	config_create_tool_cli = self.node.create_client(ConfigCreateTool, "tool/config_create_tool")
	# 	config_create_tool_req = ConfigCreateTool.Request()
	# 	config_create_tool_req.name = "tool2"
	# 	config_create_tool_req.weight = 5.3
	# 	config_create_tool_req.cog = [10.0, 10.0, 10.0]
	# 	config_create_tool_req.inertia = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	# 	config_create_tool_future = config_create_tool_cli.call_async(config_create_tool_req)
	# 	rclpy.spin_until_future_complete(self.node, config_create_tool_future, timeout_sec=SRV_CALL_TIMEOUT)
	# 	self.assertTrue(config_create_tool_future.done(), "tool/config_create_tool service working is not done.")
	# 	config_create_tool_resp = config_create_tool_future.result()
	# 	self.assertTrue(config_create_tool_resp.success == True, "tool/config_create_tool service is not working correctly.")
	# 	self.node.destroy_client(config_create_tool_resp)

	# 	time.sleep(5) ### Note: we added sleep to make sure set_current_tool works (TODO: Fix service)

	# 	""" Config Delete Tool """
	# 	config_delete_tool_cli = self.node.create_client(ConfigDeleteTool, "tool/config_delete_tool")
	# 	config_delete_tool_future = config_delete_tool_cli.call_async(ConfigDeleteTool.Request(name="tool2"))
	# 	rclpy.spin_until_future_complete(self.node, config_delete_tool_future, timeout_sec=SRV_CALL_TIMEOUT)
	# 	self.assertTrue(config_delete_tool_future.done(), "tool/config_delete_tool service working is not done.")
	# 	config_delete_tool_resp = config_delete_tool_future.result()
	# 	self.assertTrue(config_delete_tool_resp.success == True, "tool/config_delete_tool service is not working correctly.")
	# 	self.node.destroy_client(config_delete_tool_resp)


# """ Force Service Client Test Class """
# class TestDsrForceCtrlCli(unittest.TestCase):
# 	@classmethod
# 	def setUpClass(cls):
# 		print("====================================================")
# 		print("===== DooSanSystem Force Tests Start ================")
# 		cls._lock = threading.Lock()
# 		cls.node =rclpy.create_node("dsr_force_control_node", namespace=NAMESPACE)


# 	@classmethod
# 	def tearDownClass(cls):
# 		print("===== DooSanSystem Force Tests Ends ================")
# 		print("====================================================")

# 	def setUp(self):
# 		print("Ready For Force Test!!")
# 		TestDsrForceCtrlCli._lock.acquire()

# 	def tearDown(self):
# 		print("Clear Force Test !!")
# 		TestDsrForceCtrlCli._lock.release()


# 	# Align Axis 1 Test
# 	def test_align_axis_1_cli(self):
# 		print("Align Axis 1 Client Test are starting...") # Debug

# 		""" Move Joint """
# 		move_cli = self.node.create_client(MoveJoint, "motion/move_joint")
# 		target_pos = [0.0, 0.0, 45.0, 0.0, 90.0, 0.0]
# 		move_req = MoveJoint.Request()
# 		move_req.pos = target_pos
# 		move_req.time = 5.0
# 		move_future = move_cli.call_async(move_req)
# 		rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(move_future.done(), "motion/move joint future task")
# 		move_resp = move_future.result()
# 		self.assertTrue(move_resp.success == True, "motion/move joint response")
# 		self.node.destroy_client(move_cli)

# 		""" Align Axis 1 """
# 		x1 = [0.0, 500.0, 700.0, 30.0, 0.0, 0.0]
# 		x2 = [500.0, 0.0, 700.0, 0.0, 0.0, 0.0]
# 		x3 = [300.0, 100.0, 500.0, 0.0, 0.0, 0.0]

# 		pos = [400.0, 400.0, 500.0]

# 		align_axis_1_cli = self.node.create_client(AlignAxis1, "force/align_axis1")
# 		align_axis_1_req = AlignAxis1.Request()
# 		align_axis_1_req.x1 = x1
# 		align_axis_1_req.x2 = x2
# 		align_axis_1_req.x3 = x3
# 		align_axis_1_req.source_vect = pos
# 		align_axis_1_req.axis = 0
# 		align_axis_1_req.ref = 0
# 		align_axis_1_future = align_axis_1_cli.call_async(align_axis_1_req)
# 		rclpy.spin_until_future_complete(self.node, align_axis_1_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(align_axis_1_future.done(), "force/align_axis1")
# 		align_axis_1_resp = align_axis_1_future.result()
# 		self.assertTrue(align_axis_1_resp.success == True, "force/align_axis1")
# 		self.node.destroy_client(align_axis_1_cli)

# 		time.sleep(10)

# 		""" Get Current Posx """
# 		get_current_posx_cli = self.node.create_client(GetCurrentPosx, "aux_control/get_current_posx")
# 		get_current_posx_future = get_current_posx_cli.call_async(GetCurrentPosx.Request())
# 		rclpy.spin_until_future_complete(self.node, get_current_posx_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(get_current_posx_future.done(), "aux_control/get_current_posj service working is not done.")
# 		get_current_posx_resp = get_current_posx_future.result()
# 		self.assertTrue(get_current_posx_resp.success == True, "aux_control/get_current_posj service is not working correctly.")
# 		self.node.destroy_client(get_current_posx_cli)
		
# 		get_current_posx_result = get_current_posx_resp.task_pos_info[0].data
# 		# print(get_current_posx_result) # Debug
# 		self.assertAlmostEqual(get_current_posx_result[0], pos[0], delta = 0.1)
# 		self.assertAlmostEqual(get_current_posx_result[1], pos[1], delta = 0.1)
# 		self.assertAlmostEqual(get_current_posx_result[2], pos[2], delta = 0.1)

# 		""" Move Joint """
# 		move_cli = self.node.create_client(MoveJoint, "motion/move_joint")
# 		target_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 		move_req = MoveJoint.Request()
# 		move_req.pos = target_pos
# 		move_req.time = 5.0
# 		move_future = move_cli.call_async(move_req)
# 		rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(move_future.done(), "motion/move joint future task")
# 		move_resp = move_future.result()
# 		self.assertTrue(move_resp.success == True, "motion/move joint response")
# 		self.node.destroy_client(move_cli)
	

# 	# Align Axis 2 Test
# 	def test_align_axis_2_cli(self):
# 		print("Align Axis 2 Client Test are starting...") # Debug

# 		""" Move Joint """
# 		move_cli = self.node.create_client(MoveJoint, "motion/move_joint")
# 		target_pos = [0.0, 0.0, 45.0, 0.0, 90.0, 0.0]
# 		move_req = MoveJoint.Request()
# 		move_req.pos = target_pos
# 		move_req.time = 5.0
# 		move_future = move_cli.call_async(move_req)
# 		rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(move_future.done(), "motion/move joint future task")
# 		move_resp = move_future.result()
# 		self.assertTrue(move_resp.success == True, "motion/move joint response")
# 		self.node.destroy_client(move_cli)

# 		""" Align Axis 2 """
# 		align_axis_2_cli = self.node.create_client(AlignAxis2, "force/align_axis2")
# 		align_axis_2_req = AlignAxis2.Request()
# 		align_axis_2_req.target_vect = [400.0, 400.0, 500.0]
# 		align_axis_2_req.source_vect = [350.0, 37.0, 430.0]
# 		align_axis_2_req.axis = 0
# 		align_axis_2_req.ref = 0
# 		align_axis_2_future = align_axis_2_cli.call_async(align_axis_2_req)
# 		rclpy.spin_until_future_complete(self.node, align_axis_2_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(align_axis_2_future.done(), "force/align_axis2")
# 		align_axis_2_resp = align_axis_2_future.result()
# 		self.assertTrue(align_axis_2_resp.success == True, "force/align_axis2")
# 		self.node.destroy_client(align_axis_2_cli)

# 		time.sleep(10)

# 		""" Get Current Posx """
# 		get_current_posx_cli = self.node.create_client(GetCurrentPosx, "aux_control/get_current_posx")
# 		get_current_posx_future = get_current_posx_cli.call_async(GetCurrentPosx.Request())
# 		rclpy.spin_until_future_complete(self.node, get_current_posx_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(get_current_posx_future.done(), "aux_control/get_current_posj service working is not done.")
# 		get_current_posx_resp = get_current_posx_future.result()
# 		self.assertTrue(get_current_posx_resp.success == True, "aux_control/get_current_posj service is not working correctly.")
# 		self.node.destroy_client(get_current_posx_cli)
		
# 		get_current_posx_result = get_current_posx_resp.task_pos_info[0].data
# 		# print(get_current_posx_result) # Debug
# 		self.assertAlmostEqual(get_current_posx_result[0], 350.0, delta = 0.1)
# 		self.assertAlmostEqual(get_current_posx_result[1], 37.0, delta = 0.1)
# 		self.assertAlmostEqual(get_current_posx_result[2], 430.0, delta = 0.1)

# 		""" Move Joint """
# 		move_cli = self.node.create_client(MoveJoint, "motion/move_joint")
# 		target_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 		move_req = MoveJoint.Request()
# 		move_req.pos = target_pos
# 		move_req.time = 5.0
# 		move_future = move_cli.call_async(move_req)
# 		rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(move_future.done(), "motion/move joint future task")
# 		move_resp = move_future.result()
# 		self.assertTrue(move_resp.success == True, "motion/move joint response")
# 		self.node.destroy_client(move_cli)


# 	# Set User Cartesian Coordinate 1 Test
# 	def test_set_user_cart_coord1_cli(self):
# 		print("Set User Cartesian Coordinate 1 Client Test are starting...") # Debug

# 		""" Set User Cartesian Coordinate 1 """
# 		task_pos = [500.0, 30.0, 500.0, 0.0, 0.0, 0.0]
# 		set_user_cart_coord1_cli = self.node.create_client(SetUserCartCoord1, "force/set_user_cart_coord1")
# 		set_user_cart_coord1_req = SetUserCartCoord1.Request()
# 		set_user_cart_coord1_req.pos = task_pos
# 		set_user_cart_coord1_req.ref = 0
# 		set_user_cart_coord1_future = set_user_cart_coord1_cli.call_async(set_user_cart_coord1_req)
# 		rclpy.spin_until_future_complete(self.node, set_user_cart_coord1_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(set_user_cart_coord1_future.done(), "force/set_user_cart_coord1 service working is not done.")
# 		set_user_cart_coord1_resp = set_user_cart_coord1_future.result()
# 		self.assertTrue(set_user_cart_coord1_resp.success == True, "force/set_user_cart_coord1 service is not working correctly.")
# 		self.node.destroy_client(set_user_cart_coord1_cli)

# 		time.sleep(5)

# 		""" Get User Cartesian Coordinate """
# 		get_user_cart_coord_cli = self.node.create_client(GetUserCartCoord, "force/get_user_cart_coord")
# 		get_user_cart_coord_req = GetUserCartCoord.Request()
# 		get_user_cart_coord_req.id = set_user_cart_coord1_resp.id
# 		get_user_cart_coord_future = get_user_cart_coord_cli.call_async(get_user_cart_coord_req)
# 		rclpy.spin_until_future_complete(self.node, get_user_cart_coord_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(get_user_cart_coord_future.done(), "force/get_user_cart_coord service working is not done.")
# 		get_user_cart_coord_resp = get_user_cart_coord_future.result()
# 		self.assertTrue(get_user_cart_coord_resp.success == True, "force/get_user_cart_coord service is not working correctly.")
# 		self.node.destroy_client(get_user_cart_coord_cli)

# 		""" Check User Cartesian Coordinate """
# 		user_cart_coord = get_user_cart_coord_resp.conv_posx
# 		# print(user_cart_coord) # Debug
# 		self.assertAlmostEqual(task_pos[0], user_cart_coord[0], delta=0.1)
# 		self.assertAlmostEqual(task_pos[1], user_cart_coord[1], delta=0.1)
# 		self.assertAlmostEqual(task_pos[2], user_cart_coord[2], delta=0.1)
# 		self.assertAlmostEqual(task_pos[3], user_cart_coord[3], delta=0.1)
# 		self.assertAlmostEqual(task_pos[4], user_cart_coord[4], delta=0.1)
# 		self.assertAlmostEqual(task_pos[5], user_cart_coord[5], delta=0.1)


# 	# Set User Cartesian Coordinate 2 Test
# 	def test_set_user_cart_coord2_cli(self):
# 		print("Set User Cartesian Coordinate 2 Client Test are starting...") # Debug

# 		""" Set User Cartesian Coordinate 2 """
# 		x1 = [0.0, 500.0, 700.0, 0.0, 0.0, 0.0]
# 		x2 = [500.0, 0.0, 700.0, 0.0, 0.0, 0.0]
# 		x3 = [300.0, 100.0, 500.0, 0.0, 0.0, 0.0]

# 		origin = [10.0, 20.0, 30.0, 0.0, 0.0, 0.0]

# 		set_user_cart_coord2_cli = self.node.create_client(SetUserCartCoord2, "force/set_user_cart_coord2")
# 		set_user_cart_coord2_req = SetUserCartCoord2.Request()
# 		set_user_cart_coord2_req.x1 = x1
# 		set_user_cart_coord2_req.x2 = x2
# 		set_user_cart_coord2_req.x3 = x3
# 		set_user_cart_coord2_req.ref = 0
# 		set_user_cart_coord2_req.pos = origin 
# 		set_user_cart_coord2_future = set_user_cart_coord2_cli.call_async(set_user_cart_coord2_req)
# 		rclpy.spin_until_future_complete(self.node, set_user_cart_coord2_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(set_user_cart_coord2_future.done(), "force/set_user_cart_coord2 service working is not done.")
# 		set_user_cart_coord2_resp = set_user_cart_coord2_future.result()
# 		self.assertTrue(set_user_cart_coord2_resp.success == True, "force/set_user_cart_coord2 service is not working correctly.")
# 		self.node.destroy_client(set_user_cart_coord2_cli)

# 		time.sleep(5)

# 		""" Get User Cartesian Coordinate """
# 		get_user_cart_coord_cli = self.node.create_client(GetUserCartCoord, "force/get_user_cart_coord")
# 		get_user_cart_coord_req = GetUserCartCoord.Request()
# 		get_user_cart_coord_req.id = set_user_cart_coord2_resp.id
# 		get_user_cart_coord_future = get_user_cart_coord_cli.call_async(get_user_cart_coord_req)
# 		rclpy.spin_until_future_complete(self.node, get_user_cart_coord_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(get_user_cart_coord_future.done(), "force/get_user_cart_coord service working is not done.")
# 		get_user_cart_coord_resp = get_user_cart_coord_future.result()
# 		self.assertTrue(get_user_cart_coord_resp.success == True, "force/get_user_cart_coord service is not working correctly.")
# 		self.node.destroy_client(get_user_cart_coord_cli)

# 		""" Check User Cartesian Coordinate """
# 		user_cart_coord = get_user_cart_coord_resp.conv_posx
# 		# print(user_cart_coord) # Debug
# 		self.assertAlmostEqual(origin[0], user_cart_coord[0], delta=0.1)
# 		self.assertAlmostEqual(origin[1], user_cart_coord[1], delta=0.1)
# 		self.assertAlmostEqual(origin[2], user_cart_coord[2], delta=0.1)


# 	# Set User Cartesian Coordinate 3 Test
# 	def test_set_user_cart_coord3_cli(self):
# 		print("Set User Cartesian Coordinate 3 Client Test are starting...") # Debug

# 		""" Set User Cartesian Coordinate 3 """
# 		u1 = [1.0, 1.0, 0.0]
# 		v1 = [-1.0, 1.0, 0.0]

# 		origin = [10.0, 20.0, 30.0, 0.0, 0.0, 0.0]

# 		set_user_cart_coord3_cli = self.node.create_client(SetUserCartCoord3, "force/set_user_cart_coord3")
# 		set_user_cart_coord3_req = SetUserCartCoord3.Request()
# 		set_user_cart_coord3_req.u1 = u1
# 		set_user_cart_coord3_req.v1 = v1
# 		set_user_cart_coord3_req.ref = 0
# 		set_user_cart_coord3_req.pos = origin 
# 		set_user_cart_coord3_future = set_user_cart_coord3_cli.call_async(set_user_cart_coord3_req)
# 		rclpy.spin_until_future_complete(self.node, set_user_cart_coord3_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(set_user_cart_coord3_future.done(), "force/set_user_cart_coord3 service working is not done.")
# 		set_user_cart_coord3_resp = set_user_cart_coord3_future.result()
# 		self.assertTrue(set_user_cart_coord3_resp.success == True, "force/set_user_cart_coord3 service is not working correctly.")
# 		self.node.destroy_client(set_user_cart_coord3_cli)

# 		time.sleep(5)

# 		""" Get User Cartesian Coordinate """
# 		get_user_cart_coord_cli = self.node.create_client(GetUserCartCoord, "force/get_user_cart_coord")
# 		get_user_cart_coord_req = GetUserCartCoord.Request()
# 		get_user_cart_coord_req.id = set_user_cart_coord3_resp.id
# 		get_user_cart_coord_future = get_user_cart_coord_cli.call_async(get_user_cart_coord_req)
# 		rclpy.spin_until_future_complete(self.node, get_user_cart_coord_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(get_user_cart_coord_future.done(), "force/get_user_cart_coord service working is not done.")
# 		get_user_cart_coord_resp = get_user_cart_coord_future.result()
# 		self.assertTrue(get_user_cart_coord_resp.success == True, "force/get_user_cart_coord service is not working correctly.")
# 		self.node.destroy_client(get_user_cart_coord_cli)

# 		""" Check User Cartesian Coordinate """
# 		user_cart_coord = get_user_cart_coord_resp.conv_posx
# 		print(user_cart_coord) # Debug
# 		self.assertAlmostEqual(origin[0], user_cart_coord[0], delta=0.1)
# 		self.assertAlmostEqual(origin[1], user_cart_coord[1], delta=0.1)
# 		self.assertAlmostEqual(origin[2], user_cart_coord[2], delta=0.1)

	
# 	# Over Write User Cartesian Coordinate
# 	def test_overwrite_user_cart_coord_cli(self):
# 		print("Overwrite User Cartesian Coordinate Client Test are starting...") # Debug

# 		""" Set User Cartesian Coordinate 1 """
# 		task_pos = [30.0, 40.0, 50.0, 0.0, 0.0, 0.0]
# 		set_user_cart_coord1_cli = self.node.create_client(SetUserCartCoord1, "force/set_user_cart_coord1")
# 		set_user_cart_coord1_req = SetUserCartCoord1.Request()
# 		set_user_cart_coord1_req.pos = task_pos
# 		set_user_cart_coord1_req.ref = 0
# 		set_user_cart_coord1_future = set_user_cart_coord1_cli.call_async(set_user_cart_coord1_req)
# 		rclpy.spin_until_future_complete(self.node, set_user_cart_coord1_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(set_user_cart_coord1_future.done(), "force/set_user_cart_coord1 service working is not done.")
# 		set_user_cart_coord1_resp = set_user_cart_coord1_future.result()
# 		self.assertTrue(set_user_cart_coord1_resp.success == True, "force/set_user_cart_coord1 service is not working correctly.")
# 		self.node.destroy_client(set_user_cart_coord1_cli)

# 		time.sleep(5)

# 		user_coord_id = set_user_cart_coord1_resp.id

# 		""" Over Write User Cartesian Coordinate """
# 		target_pos = [100.0, 150.0, 200.0, 45.0, 180.0, 0.0]

# 		overwrite_user_cart_coord_cli = self.node.create_client(OverwriteUserCartCoord, "force/overwrite_user_cart_coord")
# 		overwrite_user_cart_coord_req = OverwriteUserCartCoord.Request()
# 		overwrite_user_cart_coord_req.id = user_coord_id
# 		overwrite_user_cart_coord_req.pos = target_pos
# 		overwrite_user_cart_coord_req.ref = 0
# 		overwrite_user_cart_coord_future = overwrite_user_cart_coord_cli.call_async(overwrite_user_cart_coord_req)
# 		rclpy.spin_until_future_complete(self.node, overwrite_user_cart_coord_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(overwrite_user_cart_coord_future.done(), "force/overwrite_user_cart_coord service working is not done.")
# 		overwrite_user_cart_coord_resp = overwrite_user_cart_coord_future.result()
# 		self.assertTrue(overwrite_user_cart_coord_resp.success == True, "force/overwrite_user_cart_coord service is not working correctly.")
# 		self.node.destroy_client(overwrite_user_cart_coord_cli)

# 		""" Get User Cartesian Coordinate """
# 		get_user_cart_coord_cli = self.node.create_client(GetUserCartCoord, "force/get_user_cart_coord")
# 		get_user_cart_coord_req = GetUserCartCoord.Request()
# 		get_user_cart_coord_req.id = overwrite_user_cart_coord_resp.id
# 		get_user_cart_coord_future = get_user_cart_coord_cli.call_async(get_user_cart_coord_req)
# 		rclpy.spin_until_future_complete(self.node, get_user_cart_coord_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(get_user_cart_coord_future.done(), "force/get_user_cart_coord service working is not done.")
# 		get_user_cart_coord_resp = get_user_cart_coord_future.result()
# 		self.assertTrue(get_user_cart_coord_resp.success == True, "force/get_user_cart_coord service is not working correctly.")
# 		self.node.destroy_client(get_user_cart_coord_cli)

# 		""" Check User Cartesian Coordinate """
# 		user_cart_coord = get_user_cart_coord_resp.conv_posx
# 		print(user_cart_coord) # Debug
# 		self.assertAlmostEqual(target_pos[0], user_cart_coord[0], delta=0.1)
# 		self.assertAlmostEqual(target_pos[1], user_cart_coord[1], delta=0.1)
# 		self.assertAlmostEqual(target_pos[2], user_cart_coord[2], delta=0.1)
# 		self.assertAlmostEqual(target_pos[3], user_cart_coord[3], delta=0.1)
# 		self.assertAlmostEqual(target_pos[4], user_cart_coord[4], delta=0.1)
# 		self.assertAlmostEqual(target_pos[5], user_cart_coord[5], delta=0.1)

	
# 	""" TODO : Operating in API but not operating in ROS 2 """
# 	# Calculate Coordinate Test
# 	def test_calc_coord_cli(self):
# 		print("Calculate Coordinate Client Test are starting...") # Debug

# 		""" Calc Coord for One input pose """
# 		x1 = [500.0, 30.0, 500.0, 0.0, 0.0, 0.0]
# 		x2 = [400.0, 30.0, 500.0, 0.0, 0.0, 0.0]
# 		x3 = [500.0, 30.0, 600.0, 45.0, 180.0, 45.0]
# 		x4 = [500.0, -30.0, 600.0, 0.0, 180.0, 0.0]

# 		calc_coord_cli = self.node.create_client(CalcCoord, "force/calc_coord")
# 		calc_coord_cli_req = CalcCoord.Request()
# 		calc_coord_cli_req.input_pos_cnt = 4
# 		calc_coord_cli_req.x1 = x1
# 		calc_coord_cli_req.x2 = x2
# 		calc_coord_cli_req.x3 = x3
# 		calc_coord_cli_req.x4 = x4
# 		calc_coord_cli_req.ref = 0
# 		calc_coord_cli_future = calc_coord_cli.call_async(calc_coord_cli_req)
# 		rclpy.spin_until_future_complete(self.node, calc_coord_cli_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(calc_coord_cli_future.done(), "force/calc_coord service working is not done.")
# 		calc_coord_cli_resp = calc_coord_cli_future.result()
# 		self.assertTrue(calc_coord_cli_resp.success == True, "force/calc_coord service is not working correctly.")
# 		self.node.destroy_client(calc_coord_cli)
	
# 		time.sleep(5)

# 		# Check Coordinate
# 		expected_result = [500, -30, 600, 90, 90, 90]
# 		print(calc_coord_cli_resp.conv_posx) # Debug
# 		self.assertAlmostEqual(expected_result[0], calc_coord_cli_resp.conv_posx[0], delta=0.1)
# 		self.assertAlmostEqual(expected_result[1], calc_coord_cli_resp.conv_posx[1], delta=0.1)
# 		self.assertAlmostEqual(expected_result[2], calc_coord_cli_resp.conv_posx[2], delta=0.1)
# 		self.assertAlmostEqual(expected_result[3], calc_coord_cli_resp.conv_posx[3], delta=0.1)
# 		self.assertAlmostEqual(expected_result[4], calc_coord_cli_resp.conv_posx[4], delta=0.1)
# 		self.assertAlmostEqual(expected_result[5], calc_coord_cli_resp.conv_posx[5], delta=0.1)
	
# 	""" TODO : Operating in API but not operating in ROS 2 """
# 	# Check Force Condition Test
# 	def test_check_force_condition_cli(self):
# 		print("Check Force Condition Client Test are starting...") # Debug

# 		check_force_cond_cli = self.node.create_client(CheckForceCondition, "force/check_force_condition")
# 		check_force_cond_req = CheckForceCondition.Request()
# 		check_force_cond_req.axis = 2
# 		check_force_cond_req.min = 5.0
# 		check_force_cond_req.max = 10.0
# 		check_force_cond_req.ref = 2
# 		check_force_cond_future = check_force_cond_cli.call_async(check_force_cond_req)
# 		rclpy.spin_until_future_complete(self.node, check_force_cond_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(check_force_cond_future.done(), "force/check_force_condition service working is not done.")
# 		check_force_cond_resp = check_force_cond_future.result()
# 		self.assertTrue(check_force_cond_resp.success == True, "force/check_force_condition service is not working correctly.")
# 		self.node.destroy_client(check_force_cond_cli)


# 	# Check Orientation Condition 1 Test
# 	def test_check_orientation_condition1_cli(self):
# 		print("Check Orientation Condition 1 Client Test are starting...") # Debug

# 		""" Move JointX """
# 		cli = self.node.create_client(MoveJointx, "motion/move_jointx")
# 		target_pos = [400.0, 500.0, 800.0, 0.0, 180.0, 40.0]
# 		req = MoveJointx.Request()
# 		req.pos = target_pos
# 		req.vel = 30.0
# 		req.acc = 60.0
# 		future = cli.call_async(req)
# 		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(future.done(), "motion/move_jointx future task")
# 		resp = future.result()
# 		self.assertTrue(resp.success == True, "motion/move_jointx response")
# 		self.node.destroy_client(cli)

# 		# time.sleep(10)

# 		""" Check Orientation Condition 1 """
# 		check_ori_cond_cli = self.node.create_client(CheckOrientationCondition1,"force/check_orientation_condition1")
# 		check_ori_cond_req = CheckOrientationCondition1.Request()
# 		check_ori_cond_req.axis = 12
# 		check_ori_cond_req.min = [400.0, 500.0, 500.0, 0.0, 180.0, 30.0]
# 		check_ori_cond_req.max = [400.0, 500.0, 500.0, 0.0, 180.0, 60.0]
# 		check_ori_cond_future = check_ori_cond_cli.call_async(check_ori_cond_req)
# 		rclpy.spin_until_future_complete(self.node, check_ori_cond_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(check_ori_cond_future.done(), "force/check_orientaion_condition1 service working is not done.")
# 		check_ori_cond_resp = check_ori_cond_future.result()
# 		self.assertTrue(check_ori_cond_resp.success == True, "force/check_orientaion_condition1 service is not working correctly.")
# 		self.node.destroy_client(check_ori_cond_cli)


# 	# Check Orientation Condition 2 Test
# 	def test_check_orientation_condition2_cli(self):
# 		print("Check Orientation Condition 2 Client Test are starting...") # Debug

# 		""" Move JointX """
# 		cli = self.node.create_client(MoveJointx, "motion/move_jointx")
# 		target_pos = [400.0, 500.0, 800.0, 0.0, 180.0, 40.0]
# 		req = MoveJointx.Request()
# 		req.pos = target_pos
# 		req.vel = 30.0
# 		req.acc = 60.0
# 		future = cli.call_async(req)
# 		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(future.done(), "motion/move_jointx future task")
# 		resp = future.result()
# 		self.assertTrue(resp.success == True, "motion/move_jointx response")
# 		self.node.destroy_client(cli)

# 		# time.sleep(10)

# 		""" Check Orientation Condition 2 """
# 		check_ori_cond2_cli = self.node.create_client(CheckOrientationCondition2,"force/check_orientation_condition2")
# 		check_ori_cond2_req = CheckOrientationCondition2.Request()
# 		check_ori_cond2_req.axis = 10
# 		check_ori_cond2_req.min = -5.0
# 		check_ori_cond2_req.max = 10.0
# 		check_ori_cond2_req.pos = [400.0, 500.0, 800.0, 50.0, 180.0, 0.0]
# 		check_ori_cond2_future = check_ori_cond2_cli.call_async(check_ori_cond2_req)
# 		rclpy.spin_until_future_complete(self.node, check_ori_cond2_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(check_ori_cond2_future.done(), "force/check_orientaion_condition1 service working is not done.")
# 		check_ori_cond2_resp = check_ori_cond2_future.result()
# 		self.assertTrue(check_ori_cond2_resp.success == True, "force/check_orientaion_condition1 service is not working correctly.")
# 		self.node.destroy_client(check_ori_cond2_cli)


# 	# Check Position Condition Test
# 	def test_check_position_condition_cli(self):
# 		print("Check Position Condition Client Test are starting...") # Debug

# 		""" Move JointX """
# 		cli = self.node.create_client(MoveJointx, "motion/move_jointx")
# 		target_pos = [400.0, 500.0, 800.0, 0.0, 180.0, 40.0]
# 		req = MoveJointx.Request()
# 		req.pos = target_pos
# 		req.vel = 30.0
# 		req.acc = 60.0
# 		future = cli.call_async(req)
# 		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(future.done(), "motion/move_jointx future task")
# 		resp = future.result()
# 		self.assertTrue(resp.success == True, "motion/move_jointx response")
# 		self.node.destroy_client(cli)

# 		# time.sleep(10)

# 		""" Check Position Condition """
# 		check_pos_cond_cli = self.node.create_client(CheckPositionCondition,"force/check_position_condition")
# 		check_pos_cond_req = CheckPositionCondition.Request()
# 		check_pos_cond_req.axis = 1
# 		check_pos_cond_req.min = -10.0
# 		check_pos_cond_req.max = 10.0
# 		check_pos_cond_req.pos = [400.0, 500.0, 800.0, 60.0, 180.0, 100.0]
# 		check_pos_cond_future = check_pos_cond_cli.call_async(check_pos_cond_req)
# 		rclpy.spin_until_future_complete(self.node, check_pos_cond_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(check_pos_cond_future.done(), "force/check_position_condition service working is not done.")
# 		check_pos_cond_resp = check_pos_cond_future.result()
# 		self.assertTrue(check_pos_cond_resp.success == True, "force/check_position_condition service is not working correctly.")
# 		self.node.destroy_client(check_pos_cond_cli)


# 	""" TODO : Do not create Coordinate Transform Result """
# 	# Coordinate Transform Test
# 	def test_coord_trans_cli(self):
# 		print("Coordinate Transform Client Test are starting...") # Debug
	
# 		""" Coordinate Trasform """
# 		base_pose = [400.0, 500.0, 800.0, 0.0, 180.0, 15.0]

# 		coord_trans_cli = self.node.create_client(CoordTransform, "force/coord_transform")
# 		coord_trans_req = CoordTransform.Request()
# 		coord_trans_req.pos_in = base_pose
# 		coord_trans_req.ref_in = 0
# 		coord_trans_req.ref_out = 1
# 		coord_trans_future = coord_trans_cli.call_async(coord_trans_req)
# 		rclpy.spin_until_future_complete(self.node, coord_trans_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(coord_trans_future.done(), "force/coord_transform service working is not done.")
# 		coord_trans_resp = coord_trans_future.result()
# 		self.assertTrue(coord_trans_resp.success == True, "force/coord_transform service is not working correctly.")
# 		self.node.destroy_client(coord_trans_cli)
	
# 		# Check Result
# 		resp_coord = coord_trans_resp.conv_posx
# 		print(resp_coord) # Debug
# 		result_coord = [400.0, 465.0, -652.5, 0.0, 180.0, 15.0] # Pre Calculate
		
# 		self.assertAlmostEqual(result_coord[0], resp_coord[0],delta=0.1)
# 		self.assertAlmostEqual(result_coord[1], resp_coord[1],delta=0.1)
# 		self.assertAlmostEqual(result_coord[2], resp_coord[2],delta=0.1)
# 		self.assertAlmostEqual(result_coord[3], resp_coord[3],delta=0.1)
# 		self.assertAlmostEqual(result_coord[4], resp_coord[4],delta=0.1)
# 		self.assertAlmostEqual(result_coord[5], resp_coord[5],delta=0.1)


# 	# Get Work Piece Weight Test 
# 	def test_get_workpiece_weight_cli(self):
# 		print("Get Workpiece Weight Client Test are starting...") # Debug

# 		""" Get Work Piece Weight """
# 		get_workpiece_weight_cli = self.node.create_client(GetWorkpieceWeight, "force/get_workpiece_weight")
# 		get_workpiece_weight_future = get_workpiece_weight_cli.call_async(GetWorkpieceWeight.Request())
# 		rclpy.spin_until_future_complete(self.node, get_workpiece_weight_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(get_workpiece_weight_future.done(), "force/get_workpiece_weight service working is not done.")
# 		get_workpiece_weight_resp = get_workpiece_weight_future.result()
# 		self.assertTrue((get_workpiece_weight_resp.success == True) and (get_workpiece_weight_resp.weight == 0.0), "force/get_workpiece_weight service is not working correctly.")
# 		self.node.destroy_client(get_workpiece_weight_cli)


# 	# Reset Workpiece Weight Test
# 	def test_reset_workpiece_weight_cli(self):
# 		print("Reset Workpiece Weight Client Test are starting...") # Debug

# 		""" Reset Workpiece Weight """
# 		reset_workpiece_weight_cli = self.node.create_client(ResetWorkpieceWeight, "force/reset_workpiece_weight")
# 		reset_workpiece_weight_future = reset_workpiece_weight_cli.call_async(ResetWorkpieceWeight.Request())
# 		rclpy.spin_until_future_complete(self.node, reset_workpiece_weight_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(reset_workpiece_weight_future.done(), "force/reset_workpiece_weight service working is not done.")
# 		reset_workpiece_weight_resp = reset_workpiece_weight_future.result()
# 		self.assertTrue(reset_workpiece_weight_resp.success == True, "force/reset_workpiece_weight service is not working correctly.")
# 		self.node.destroy_client(reset_workpiece_weight_cli)


# 	# Is Done Bolt Tightening Test
# 	def test_is_done_bolt_tightening_cli(self):
# 		print("Is Done Bolt Tightening Client Test are starting...") # Debug

# 		""" Move Joint """
# 		cli = self.node.create_client(MoveJoint, "motion/move_joint")
# 		target_pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
# 		req = MoveJoint.Request()
# 		req.pos = target_pos
# 		req.time = 5.0
# 		future = cli.call_async(req)
# 		rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(future.done(), "motion/move_joint service working is not done.")
# 		resp = future.result()
# 		self.assertTrue(resp.success == True, "motion/move_joint service is not working correctly.")
# 		self.node.destroy_client(cli)

# 		# time.sleep(10)

# 		""" Taks Compliance Control """
# 		task_compliance_ctrl_cli = self.node.create_client(TaskComplianceCtrl, "force/task_compliance_ctrl")
# 		task_compliance_ctrl_req = TaskComplianceCtrl.Request()
# 		task_compliance_ctrl_req.stx = [3000.0, 3000.0, 3000.0, 200.0, 200.0, 200.0]
# 		task_compliance_ctrl_req.ref = 0
# 		task_compliance_ctrl_req.time = 1.0
# 		task_compliance_ctrl_future = task_compliance_ctrl_cli.call_async(task_compliance_ctrl_req)
# 		rclpy.spin_until_future_complete(self.node, task_compliance_ctrl_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(task_compliance_ctrl_future.done(), "force/task_compliance_ctrl service working is not done.")
# 		task_compliance_ctrl_resp = task_compliance_ctrl_future.result()
# 		self.assertTrue(task_compliance_ctrl_resp.success == True, "force/task_compliance_ctrl service is not working correctly.")
# 		self.node.destroy_client(task_compliance_ctrl_cli)


# 		""" Move line """
# 		target_pos = [559.0, 34.5, 651.5, 0.0, 180.0, 60.0]

# 		move_line_cli = self.node.create_client(MoveLine, "motion/move_line")
# 		move_line_req = MoveLine.Request()
# 		move_line_req.pos = target_pos
# 		move_line_req.vel = [50.0, 50.0]
# 		move_line_req.acc = [50.0, 50.0]
# 		move_line_req.sync_type = 1
# 		move_line_future = move_line_cli.call_async(move_line_req)
# 		rclpy.spin_until_future_complete(self.node, move_line_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(move_line_future.done(), "motion/move_line service working is not done.")
# 		move_line_resp = move_line_future.result()
# 		self.assertTrue(move_line_resp.success == True, "motion/move_line service is not working correctly.")
# 		self.node.destroy_client(move_line_cli)

# 		""" Is Done Bolt Tightening """
# 		is_done_bolt_tightening_cli = self.node.create_client(IsDoneBoltTightening, "force/is_done_bolt_tightening")
# 		is_done_bolt_tightening_req = IsDoneBoltTightening.Request()
# 		is_done_bolt_tightening_req.m = 10.0
# 		is_done_bolt_tightening_req.timeout = 5.0
# 		is_done_bolt_tightening_req.axis = 2
# 		is_done_bolt_tightening_future = is_done_bolt_tightening_cli.call_async(is_done_bolt_tightening_req)
# 		rclpy.spin_until_future_complete(self.node, is_done_bolt_tightening_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(is_done_bolt_tightening_future.done(), "force/is_done_bolt_tightening service working is not done.")
# 		is_done_bolt_tightening_resp = is_done_bolt_tightening_future.result()
# 		self.assertTrue(is_done_bolt_tightening_resp.success == False, "force/is_done_bolt_tightening service is working correctly.")
# 		self.node.destroy_client(is_done_bolt_tightening_cli)


""" IO Control Service Client Test Class """
class TestDsrIOCtrlCli(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		print("====================================================")
		print("===== DooSanSystem IO Tests Start ================")
		cls._lock = threading.Lock()
		cls.node =rclpy.create_node("dsr_io_test_node", namespace=NAMESPACE)

	@classmethod
	def tearDownClass(cls):
		print("===== DooSanSystem IO Tests Ends ================")
		print("====================================================")

	def setUp(self):
		print("Ready For IO Test!!")
		TestDsrIOCtrlCli._lock.acquire()

	def tearDown(self):
		print("Clear For IO Test!!")
		TestDsrIOCtrlCli._lock.release()

	# Set Control Box Analog Input Type
	def test_set_ctrl_box_analog_input_type_cli(self):
		print("Set Control Box Analog Input Type Client Test are starting...") # Debug

		""" Set Control Box Analog Input Type """
		set_crtl_box_analog_input_type_cli = self.node.create_client(SetCtrlBoxAnalogInputType, "io/set_ctrl_box_analog_input_type")
		set_crtl_box_analog_input_type_req = SetCtrlBoxAnalogInputType.Request()
		set_crtl_box_analog_input_type_req.channel = 1
		set_crtl_box_analog_input_type_req.mode = 0
		set_crtl_box_analog_input_type_future = set_crtl_box_analog_input_type_cli.call_async(set_crtl_box_analog_input_type_req)
		rclpy.spin_until_future_complete(self.node, set_crtl_box_analog_input_type_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_crtl_box_analog_input_type_future.done(), "io/set_ctrl_box_analog_input_type service working is not done.")
		set_crtl_box_analog_input_type_resp = set_crtl_box_analog_input_type_future.result()
		self.assertTrue(set_crtl_box_analog_input_type_resp.success == True, "io/set_ctrl_box_analog_input_type service is not working correctly.")
		self.node.destroy_client(set_crtl_box_analog_input_type_cli)


	# Get Control Box Analog Input
	def test_get_ctrl_box_analog_input_cli(self):
		print("Get Control Box Analog Input Client Test are starting...") # Debug

		""" Set Control Box Analog Input Type """
		set_crtl_box_analog_input_type_cli = self.node.create_client(SetCtrlBoxAnalogInputType, "io/set_ctrl_box_analog_input_type")
		set_crtl_box_analog_input_type_req = SetCtrlBoxAnalogInputType.Request()
		set_crtl_box_analog_input_type_req.channel = 1
		set_crtl_box_analog_input_type_req.mode = 0
		set_crtl_box_analog_input_type_future = set_crtl_box_analog_input_type_cli.call_async(set_crtl_box_analog_input_type_req)
		rclpy.spin_until_future_complete(self.node, set_crtl_box_analog_input_type_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_crtl_box_analog_input_type_future.done(), "io/set_ctrl_box_analog_input_type service working is not done.")
		set_crtl_box_analog_input_type_resp = set_crtl_box_analog_input_type_future.result()
		self.assertTrue(set_crtl_box_analog_input_type_resp.success == True, "io/set_ctrl_box_analog_input_type service is not working correctly.")
		self.node.destroy_client(set_crtl_box_analog_input_type_cli)

		time.sleep(5) ### Note: we added sleep to make sure set_current_tool works (TODO: Fix service)

		""" Get Control Box Analog Input """
		get_crtl_box_analog_input_cli = self.node.create_client(GetCtrlBoxAnalogInput, "io/get_ctrl_box_analog_input")
		get_crtl_box_analog_input_req = GetCtrlBoxAnalogInput.Request()
		get_crtl_box_analog_input_req.channel = 1
		get_crtl_box_analog_input_future = get_crtl_box_analog_input_cli.call_async(get_crtl_box_analog_input_req)
		rclpy.spin_until_future_complete(self.node, get_crtl_box_analog_input_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_crtl_box_analog_input_future.done(), "io/get_ctrl_box_analog_input service working is not done.")
		get_crtl_box_analog_input_resp = get_crtl_box_analog_input_future.result()
		self.assertTrue(get_crtl_box_analog_input_resp.success == True, "io/get_ctrl_box_analog_input service is not working correctly.")
		self.node.destroy_client(get_crtl_box_analog_input_cli)


	# Set Control Box Analog Output Type Test
	def test_set_ctrl_box_analog_output_type_cli(self):
		print("Set Control Box Analog Output Type Client Test are starting...") # Debug

		""" Set Control Box Analog Output Type """
		set_crtl_box_analog_output_type_cli = self.node.create_client(SetCtrlBoxAnalogOutputType, "io/set_ctrl_box_analog_output_type")
		set_crtl_box_analog_output_type_req = SetCtrlBoxAnalogOutputType.Request()
		set_crtl_box_analog_output_type_req.channel = 1
		set_crtl_box_analog_output_type_req.mode = 0
		set_crtl_box_analog_output_type_future = set_crtl_box_analog_output_type_cli.call_async(set_crtl_box_analog_output_type_req)
		rclpy.spin_until_future_complete(self.node, set_crtl_box_analog_output_type_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_crtl_box_analog_output_type_future.done(), "io/set_ctrl_box_analog_output_type service working is not done.")
		set_crtl_box_analog_output_type_resp = set_crtl_box_analog_output_type_future.result()
		self.assertTrue(set_crtl_box_analog_output_type_resp.success == True, "io/set_ctrl_box_analog_output_type service is not working correctly.")
		self.node.destroy_client(set_crtl_box_analog_output_type_cli)


	# Set Control Box Analog Output Test
	def test_set_ctrl_box_analog_ouputput_cli(self):
		print("Set Control Box Analog Output Client Test are starting...") # Debug

		""" Set Control Box Analog Output Type """
		set_crtl_box_analog_output_type_cli = self.node.create_client(SetCtrlBoxAnalogOutputType, "io/set_ctrl_box_analog_output_type")
		set_crtl_box_analog_output_type_req = SetCtrlBoxAnalogOutputType.Request()
		set_crtl_box_analog_output_type_req.channel = 1
		set_crtl_box_analog_output_type_req.mode = 0
		set_crtl_box_analog_output_type_future = set_crtl_box_analog_output_type_cli.call_async(set_crtl_box_analog_output_type_req)
		rclpy.spin_until_future_complete(self.node, set_crtl_box_analog_output_type_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_crtl_box_analog_output_type_future.done(), "io/set_ctrl_box_analog_output_type service working is not done.")
		set_crtl_box_analog_output_type_resp = set_crtl_box_analog_output_type_future.result()
		self.assertTrue(set_crtl_box_analog_output_type_resp.success == True, "io/set_ctrl_box_analog_output_type service is not working correctly.")
		self.node.destroy_client(set_crtl_box_analog_output_type_cli)

		time.sleep(5) ### Note: we added sleep to make sure set_current_tool works (TODO: Fix )

		""" Set Control Box Analog Input """
		get_crtl_box_analog_output_cli = self.node.create_client(SetCtrlBoxAnalogOutput, "io/set_ctrl_box_analog_output")
		get_crtl_box_analog_output_req = SetCtrlBoxAnalogOutput.Request()
		get_crtl_box_analog_output_req.channel = 1
		get_crtl_box_analog_output_req.value = 5.2
		get_crtl_box_analog_output_future = get_crtl_box_analog_output_cli.call_async(get_crtl_box_analog_output_req)
		rclpy.spin_until_future_complete(self.node, get_crtl_box_analog_output_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_crtl_box_analog_output_future.done(), "io/set_ctrl_box_analog_output service working is not done.")
		get_crtl_box_analog_output_resp = get_crtl_box_analog_output_future.result()
		self.assertTrue(get_crtl_box_analog_output_resp.success == True, "io/set_ctrl_box_analog_output service is not working correctly.")
		self.node.destroy_client(get_crtl_box_analog_output_cli)


	# Set Control Box Digital Output TestTCP
	def test_set_ctrl_box_digital_output_cli(self):
		print("Set Control Box Digital Output Client Test are starting...") # Debug

		""" Set Control Box Digital Output """
		set_ctrl_box_digital_output_cli = self.node.create_client(SetCtrlBoxDigitalOutput, "io/set_ctrl_box_digital_output")
		set_ctrl_box_digital_output_req = SetCtrlBoxDigitalOutput.Request()
		set_ctrl_box_digital_output_req.index = 1 # 1 ~ 16
		set_ctrl_box_digital_output_req.value = 0 # 0 : ON
		set_ctrl_box_digital_output_future = set_ctrl_box_digital_output_cli.call_async(set_ctrl_box_digital_output_req)
		rclpy.spin_until_future_complete(self.node, set_ctrl_box_digital_output_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_ctrl_box_digital_output_future.done(), "io/set_ctrl_box_digital_output service working is not done.")
		set_ctrl_box_digital_output_resp = set_ctrl_box_digital_output_future.result()
		self.assertTrue(set_ctrl_box_digital_output_resp.success == True, "io/set_ctrl_box_digital_output service is not working correctly.")
		self.node.destroy_client(set_ctrl_box_digital_output_cli)


	# Set Tool Digital Output Test
	def test_set_tool_digital_output_cli(self):
		print("Set Tool Digital Output Client Test are starting...") # Debug

		""" Set Tool Digital Output """
		set_tool_digital_output_cli = self.node.create_client(SetToolDigitalOutput, "io/set_tool_digital_output")
		set_tool_digital_output_req = SetToolDigitalOutput.Request()
		set_tool_digital_output_req.index = 1 # 1 ~ 6
		set_tool_digital_output_req.value = 0 # 0 : ON
		set_tool_digital_output_future = set_tool_digital_output_cli.call_async(set_tool_digital_output_req)
		rclpy.spin_until_future_complete(self.node, set_tool_digital_output_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(set_tool_digital_output_future.done(), "io/set_ctrl_box_digital_output service working is not done.")
		set_tool_digital_output_resp = set_tool_digital_output_future.result()
		self.assertTrue(set_tool_digital_output_resp.success == True, "io/set_ctrl_box_digital_output service is not working correctly.")
		self.node.destroy_client(set_tool_digital_output_cli)


	# Get Control Box Digital Input Test
	def test_get_ctrl_box_digital_input_cli(self):
		print("Get Control Box Digital Input Client Test are starting...") # Debug

		""" Get Control Box Digital Input """
		get_ctrl_box_digital_input_cli = self.node.create_client(GetCtrlBoxDigitalInput, "io/get_ctrl_box_digital_input")
		get_ctrl_box_digital_input_req = GetCtrlBoxDigitalInput.Request()
		get_ctrl_box_digital_input_req.index = 1 # 1 ~ 16
		get_ctrl_box_digital_input_future = get_ctrl_box_digital_input_cli.call_async(get_ctrl_box_digital_input_req)
		rclpy.spin_until_future_complete(self.node, get_ctrl_box_digital_input_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_ctrl_box_digital_input_future.done(), "io/get_ctrl_box_digital_input service working is not done.")
		get_ctrl_box_digital_input_resp = get_ctrl_box_digital_input_future.result()
		self.assertTrue(get_ctrl_box_digital_input_resp.success == True, "io/get_ctrl_box_digital_input service is not working correctly.")
		self.node.destroy_client(get_ctrl_box_digital_input_cli)


	# Get Control Box Digital Output Test
	def test_get_ctrl_box_digital_output_cli(self):
		print("Get Control Box Digital Output Client Test are starting...") # Debug

		""" Get Control Box Digital Input """
		get_ctrl_box_digital_output_cli = self.node.create_client(GetCtrlBoxDigitalOutput, "io/get_ctrl_box_digital_output")
		get_ctrl_box_digital_output_req = GetCtrlBoxDigitalOutput.Request()
		get_ctrl_box_digital_output_req.index = 1 # 1 ~ 16
		get_ctrl_box_digital_output_future = get_ctrl_box_digital_output_cli.call_async(get_ctrl_box_digital_output_req)
		rclpy.spin_until_future_complete(self.node, get_ctrl_box_digital_output_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_ctrl_box_digital_output_future.done(), "io/get_ctrl_box_digital_output service working is not done.")
		get_ctrl_box_digital_output_resp = get_ctrl_box_digital_output_future.result()
		self.assertTrue(get_ctrl_box_digital_output_resp.success == True, "io/get_ctrl_box_digital_output service is not working correctly.")
		self.node.destroy_client(get_ctrl_box_digital_output_cli)


	# Get Tool Digital Input Test
	def test_get_tool_digital_input_cli(self):
		print("Get Tool Digital Input Client Test are starting...") # Debug

		""" Get Tool Digital Input """
		get_tool_digital_input_cli = self.node.create_client(GetToolDigitalInput, "io/get_tool_digital_input")
		get_tool_digital_input_req = GetToolDigitalInput.Request()
		get_tool_digital_input_req.index = 1 # 1 ~ 6
		get_tool_digital_input_future = get_tool_digital_input_cli.call_async(get_tool_digital_input_req)
		rclpy.spin_until_future_complete(self.node, get_tool_digital_input_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_tool_digital_input_future.done(), "io/get_tool_digital_input service working is not done.")
		get_tool_digital_input_resp = get_tool_digital_input_future.result()
		self.assertTrue(get_tool_digital_input_resp.success == True, "io/get_tool_digital_input service is not working correctly.")
		self.node.destroy_client(get_tool_digital_input_cli)


	# Get Tool Digital Output Test 
	def test_get_tool_digital_output_cli(self):
		print("Get Tool Digital Output Client Test are starting...") # Debug

		""" Get Tool Digital Output """
		get_tool_digital_output_cli = self.node.create_client(GetToolDigitalOutput, "io/get_tool_digital_output")
		get_tool_digital_output_req = GetToolDigitalOutput.Request()
		get_tool_digital_output_req.index = 1 # 1 ~ 6
		get_tool_digital_output_future = get_tool_digital_output_cli.call_async(get_tool_digital_output_req)
		rclpy.spin_until_future_complete(self.node, get_tool_digital_output_future, timeout_sec=SRV_CALL_TIMEOUT)
		self.assertTrue(get_tool_digital_output_future.done(), "io/get_tool_digital_output service working is not done.")
		get_tool_digital_output_resp = get_tool_digital_output_future.result()
		self.assertTrue(get_tool_digital_output_resp.success == True, "io/get_tool_digital_output service is not working correctly.")
		self.node.destroy_client(get_tool_digital_output_cli)


# """ TCP Service Client Test Class """
# class TestDsrTCPCtrlCli(unittest.TestCase):
# 	@classmethod
# 	def setUpClass(cls):
# 		print("====================================================")
# 		print("===== DooSanSystem TCP Tests Start ================")
# 		cls._lock = threading.Lock()
# 		cls.node =rclpy.create_node("dsr_move_test_node", namespace=NAMESPACE)


# 	@classmethod
# 	def tearDownClass(cls):
# 		print("===== DooSanSystem TCP Tests Ends ================")
# 		print("====================================================")
		
# 	def setUp(self):
# 		print("Ready For TCP Test!!")
# 		TestDsrTCPCtrlCli._lock.acquire()

# 	def tearDown(self):
# 		print("Clear TCP Test!!")
# 		TestDsrTCPCtrlCli._lock.release()

# 	# Configure Create TCP Test
# 	def test_config_create_tcp_cli(self):
# 		print("Create TCP Configuration Client Test are starting...") # Debug

# 		""" Config Create TCP """
# 		config_create_tcp_cli = self.node.create_client(ConfigCreateTcp, "tcp/config_create_tcp")
# 		config_create_tcp_req = ConfigCreateTcp.Request()
# 		config_create_tcp_req.name = "tcp1"
# 		config_create_tcp_req.pos = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
# 		config_create_tcp_future = config_create_tcp_cli.call_async(config_create_tcp_req)
# 		rclpy.spin_until_future_complete(self.node, config_create_tcp_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(config_create_tcp_future.done(), "tcp/config_create_tcp service working is not done.")
# 		config_create_tcp_resp = config_create_tcp_future.result()
# 		self.assertTrue(config_create_tcp_resp.success == True, "tcp/config_create_tcp service is not working correctly.")
# 		self.node.destroy_client(config_create_tcp_cli)

	
# 	# Set Current TCP Test
# 	def test_set_current_tcp_cli(self):
# 		print("Set Current TCP Client Test are starting...") # Debug

# 		""" Config Create TCP """
# 		config_create_tcp_cli = self.node.create_client(ConfigCreateTcp, "tcp/config_create_tcp")
# 		config_create_tcp_req = ConfigCreateTcp.Request()
# 		config_create_tcp_req.name = "tcp1"
# 		config_create_tcp_req.pos = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
# 		config_create_tcp_future = config_create_tcp_cli.call_async(config_create_tcp_req)
# 		rclpy.spin_until_future_complete(self.node, config_create_tcp_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(config_create_tcp_future.done(), "tcp/config_create_tcp service working is not done.")
# 		config_create_tcp_resp = config_create_tcp_future.result()
# 		self.assertTrue(config_create_tcp_resp.success == True, "tcp/config_create_tcp service is not working correctly.")
# 		self.node.destroy_client(config_create_tcp_cli)

# 		time.sleep(5)

# 		""" Set Current TCP """
# 		set_current_tcp_cli = self.node.create_client(SetCurrentTcp, "tcp/set_current_tcp")
# 		set_current_tcp_req = SetCurrentTcp.Request()
# 		set_current_tcp_req.name = "tcp1"
# 		set_current_tcp_future = set_current_tcp_cli.call_async(set_current_tcp_req)
# 		rclpy.spin_until_future_complete(self.node, set_current_tcp_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(set_current_tcp_future.done(), "tcp/set_current_tcp service working is not done.")
# 		set_current_tcp_resp = set_current_tcp_future.result()
# 		self.assertTrue(set_current_tcp_resp.success == True, "tcp/set_current_tcp service is not working correctly.")
# 		self.node.destroy_client(set_current_tcp_cli)

	
# 	# Get Current TCP Test
# 	def test_get_current_tcp_cli(self):
# 		print("Get Current TCP Client Test are starting...") # Debug

# 		""" Config Create TCP """
# 		config_create_tcp_cli = self.node.create_client(ConfigCreateTcp, "tcp/config_create_tcp")
# 		config_create_tcp_req = ConfigCreateTcp.Request()
# 		config_create_tcp_req.name = "tcp1"
# 		config_create_tcp_req.pos = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
# 		config_create_tcp_future = config_create_tcp_cli.call_async(config_create_tcp_req)
# 		rclpy.spin_until_future_complete(self.node, config_create_tcp_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(config_create_tcp_future.done(), "tcp/config_create_tcp service working is not done.")
# 		config_create_tcp_resp = config_create_tcp_future.result()
# 		self.assertTrue(config_create_tcp_resp.success == True, "tcp/config_create_tcp service is not working correctly.")
# 		self.node.destroy_client(config_create_tcp_cli)

# 		time.sleep(5)

# 		""" Set Current TCP """
# 		set_current_tcp_cli = self.node.create_client(SetCurrentTcp, "tcp/set_current_tcp")
# 		set_current_tcp_req = SetCurrentTcp.Request()
# 		set_current_tcp_req.name = "tcp1"
# 		set_current_tcp_future = set_current_tcp_cli.call_async(set_current_tcp_req)
# 		rclpy.spin_until_future_complete(self.node, set_current_tcp_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(set_current_tcp_future.done(), "tcp/set_current_tcp service working is not done.")
# 		set_current_tcp_resp = set_current_tcp_future.result()
# 		self.assertTrue(set_current_tcp_resp.success == True, "tcp/set_current_tcp service is not working correctly.")
# 		self.node.destroy_client(set_current_tcp_cli)

# 		time.sleep(5)

# 		""" Get Current TCP """
# 		get_current_tcp_cli = self.node.create_client(GetCurrentTcp, "tcp/get_current_tcp")
# 		get_current_tcp_future = get_current_tcp_cli.call_async(GetCurrentTcp.Request())
# 		rclpy.spin_until_future_complete(self.node, get_current_tcp_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(get_current_tcp_future.done(), "tcp/get_current_tcp service working is not done.")
# 		get_current_tcp_resp = get_current_tcp_future.result()
# 		self.assertTrue((get_current_tcp_resp.success == True) and (get_current_tcp_resp.info == "tcp1"), "tcp/get_current_tcp service is not working correctly.")
# 		self.node.destroy_client(get_current_tcp_cli)


# 	# Configure Delete TCP Test
# 	def test_config_delete_tcp_cli(self):
# 		print("Delete TCP Configuration Client Test are starting...") # Debug

# 		""" Config Create TCP """
# 		config_create_tcp_cli = self.node.create_client(ConfigCreateTcp, "tcp/config_create_tcp")
# 		config_create_tcp_req = ConfigCreateTcp.Request()
# 		config_create_tcp_req.name = "tcp1"
# 		config_create_tcp_req.pos = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
# 		config_create_tcp_future = config_create_tcp_cli.call_async(config_create_tcp_req)
# 		rclpy.spin_until_future_complete(self.node, config_create_tcp_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(config_create_tcp_future.done(), "tcp/config_create_tcp service working is not done.")
# 		config_create_tcp_resp = config_create_tcp_future.result()
# 		self.assertTrue(config_create_tcp_resp.success == True, "tcp/config_create_tcp service is not working correctly.")
# 		self.node.destroy_client(config_create_tcp_cli)
		
# 		time.sleep(5)

# 		""" Set Current TCP """
# 		set_current_tcp_cli = self.node.create_client(SetCurrentTcp, "tcp/set_current_tcp")
# 		set_current_tcp_req = SetCurrentTcp.Request()
# 		set_current_tcp_req.name = "tcp1"
# 		set_current_tcp_future = set_current_tcp_cli.call_async(set_current_tcp_req)
# 		rclpy.spin_until_future_complete(self.node, set_current_tcp_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(set_current_tcp_future.done(), "tcp/set_current_tcp service working is not done.")
# 		set_current_tcp_resp = set_current_tcp_future.result()
# 		self.assertTrue(set_current_tcp_resp.success == True, "tcp/set_current_tcp service is not working correctly.")
# 		self.node.destroy_client(set_current_tcp_cli)

# 		time.sleep(5)

# 		""" Config Delete TCP """
# 		config_delete_tcp_cli = self.node.create_client(ConfigDeleteTcp, "tcp/config_delete_tcp")
# 		config_delete_tcp_req = ConfigDeleteTcp.Request()
# 		config_delete_tcp_req.name = "tcp1"
# 		config_delete_tcp_future = config_delete_tcp_cli.call_async(config_delete_tcp_req)
# 		rclpy.spin_until_future_complete(self.node, config_delete_tcp_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(config_delete_tcp_future.done(), "tcp/config_create_tcp service working is not done.")
# 		config_delete_tcp_resp = config_delete_tcp_future.result()
# 		self.assertTrue(config_delete_tcp_resp.success == True, "tcp/config_create_tcp service is not working correctly.")
# 		self.node.destroy_client(config_delete_tcp_cli)

# 		time.sleep(5)

# 		""" Get Current TCP """
# 		get_current_tcp_cli = self.node.create_client(GetCurrentTcp, "tcp/get_current_tcp")
# 		get_current_tcp_future = get_current_tcp_cli.call_async(GetCurrentTcp.Request())
# 		rclpy.spin_until_future_complete(self.node, get_current_tcp_future, timeout_sec=SRV_CALL_TIMEOUT)
# 		self.assertTrue(get_current_tcp_future.done(), "tcp/get_current_tcp service working is not done.")
# 		get_current_tcp_resp = get_current_tcp_future.result()
# 		self.assertTrue((get_current_tcp_resp.success == True) and (get_current_tcp_resp.info == ""), "tcp/get_current_tcp service is not working correctly.")
# 		self.node.destroy_client(get_current_tcp_cli)


if __name__ == '__main__':
	unittest.main()
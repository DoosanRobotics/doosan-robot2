import unittest
import subprocess
import signal
import rclpy
import threading

from dsr_msgs2.srv import MoveJoint, GetCurrentPose

NAMESPACE = "TEST_DSR"
SRV_CALL_TIMEOUT = 10

class TestDsrCli(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		# Start prerequisite launch scripts
		cls.bringup_script = subprocess.Popen([ # how to align RAII?
			"ros2", "launch", "dsr_tests",
			"dsr_bringup_without_spawner_test.launch.py",
			"mode:=virtual",
			"name:={}".format(NAMESPACE),
			"port:=25125"
			],
			stdout=subprocess.PIPE,
			stderr=subprocess.STDOUT
		)
		cls.spawner_script = subprocess.Popen([
			"ros2", "launch", "dsr_tests",
			"dsr_spawner_cli_test.launch.py",
			"name:={}".format(NAMESPACE)
			],
			stdout=subprocess.PIPE,
			stderr=subprocess.STDOUT,
			universal_newlines=True
		)
		cls._lock = threading.Lock()

		# Assume if spawners are successfully loaded, Prerequisite done.
		try:
			(stdout, _ ) = cls.spawner_script.communicate(timeout=10)
			if 2 != stdout.count('\033[92m'+"Configured and activated"):
				cls.bringup_script.send_signal(signal.SIGINT)
				cls.spawner_script.send_signal(signal.SIGINT)
				raise Exception('Failed Loading Spawner. stdout : {}'.format(stdout))
		except subprocess.TimeoutExpired as e:
			cls.bringup_script.send_signal(signal.SIGINT)
			cls.spawner_script.send_signal(signal.SIGINT)
			raise Exception('Spawner Time out !!')

		# Test Cli node
		rclpy.init()
		cls.node = rclpy.create_node("dsr_test_node", namespace=NAMESPACE)

	@classmethod
	def tearDownClass(cls):
		# Terminate Launch scripts
		cls.bringup_script.send_signal(signal.SIGINT)
		cls.spawner_script.send_signal(signal.SIGINT)
		# Test Cli node
		cls.node.destroy_node()
		rclpy.shutdown()

	def setUp(self):
		self._lock.acquire() # seems to bad syntax.

	def tearDown(self):
		self._lock.release()

	def test_move_joint_cli(self):
		cli = self.node.create_client(MoveJoint, "motion/move_joint")
		target_pos = [0., 0., 30., 0., 0., 0.]
		req = MoveJoint.Request(pos=target_pos, vel=30., acc=30.)
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


if __name__ == '__main__':
	unittest.main()
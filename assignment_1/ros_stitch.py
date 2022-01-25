#!/usr/bin/env python3

from sc627_helper.msg import MoveXYActionGoal, MoveXYActionResult
import rospy

last_pos = 0
pub = rospy.Publisher('/move_xy/goal', MoveXYActionGoal, queue_size=10)

def callback_result(data):
	print("New msg received")
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.result.pose_final)
	curr_pos = data.result.pose_final.x
	global last_pos
	last_pos = curr_pos

def listener():
	rospy.init_node('test', anonymous=True, disable_signals=True)
	rospy.Subscriber('/move_xy/result', MoveXYActionResult, callback_result)

	while not rospy.is_shutdown():
		go_to = MoveXYActionGoal()
		print(last_pos)
		if last_pos <= 1.5:
			go_to.goal.pose_dest.x = 1 + last_pos
			print(go_to.goal.pose_dest.x)
			pub.publish(go_to)
			print("Last msg published")
			rospy.sleep(20)
		else:
			rospy.signal_shutdown("Reached the goal")

if __name__=='__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
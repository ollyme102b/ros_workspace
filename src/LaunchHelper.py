
# collection of functions to help with ROS implementation

import rospy


def get_and_set_params(node_name, params):
	# params is a dict from param_name -> default_value, None if no default value
	vals = {}
	for param, default in params.items():
		if default is None:
			vals[param] = rospy.get_param("{}/{}".format(node_name, param))
		else:
			if not rospy.has_param("{}/{}".format(node_name, param)):
				rospy.set_param("{}/{}".format(node_name, param), default)
			vals[param] = rospy.get_param("{}/{}".format(node_name, param))
	return vals

def clear_params(node_name, params):
	for k in params:
		try:
			rospy.delete_param("{}/{}".format(node_name, k))
		except KeyError:
			print("value {} not set".format(k))
	return True
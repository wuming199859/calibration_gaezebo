#! /usr/bin/env python3

import rospy
#入口
if __name__ == "__main__":
   #初始化ros节点
   rospy.init_node("hello_p")
   #输出日志
   rospy.loginfo("hello!这里python!")
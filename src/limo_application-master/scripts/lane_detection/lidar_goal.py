import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import dynamic_reconfigure.client

class test():
    def __init__(self):
        self.goal_list = [[ 6.386, 0.837, 0.033],[11.053, 0.534, -0.370],[11.543, -2.072, -2.068],[9.101, -1.604, 2.236],[11.154, 0.571, -0.074],[11.711, -1.755, -1.972],[9.315, -1.870, 2.755],[7.163, -3.499, -3.113],[0.296, -4.609, -3.121]]
   #9.787 obstacle
   #7.022 -3.507 -3.038
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
        self.odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odomCB)
        self.secs = 0
        self.nsecs = 0
        self.point = 0
        self.prev_x = 0
        self.prev_y = 0
        self.x = 0
        self.y = 0
        self.q = 0 
        
    def odomCB(self, msg):
        self.secs = msg.header.stamp.secs
        self.nsecs = msg.header.stamp.nsecs
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
   

        msg_p = PoseStamped()
        msg_p.header.frame_id = 'map'
        msg_p.header.seq = 0
        msg_p.header.stamp.secs = self.secs
        msg_p.header.stamp.nsecs = self.nsecs
   

        if self.point == 0: # init waypoint => watpoint [0]
            self.point = self.point+1 # point to 1
            print(self.point)
        
            msg_p.pose.position.x = self.goal_list[0][0]
            msg_p.pose.position.y = self.goal_list[0][1]
            q = quaternion_from_euler(0, 0, self.goal_list[0][2])
            msg_p.pose.position.z = 0
            msg_p.pose.orientation.z = q[2]
            msg_p.pose.orientation.w = q[3]
            
            for i in range(1,3):
                
                print("send msg")
                self.goal_pub.publish(msg_p)
                rospy.sleep(1)
            
        else:
            pass
      

        self.prev_x = self.goal_list[self.point-1][0]   
        self.prev_y = self.goal_list[self.point-1][1]
        
        print("x:",self.x)
        print("prev_x:", self.prev_x)   
        print("y:",self.y)
        print("prev_y:", self.prev_y)

        if self.x >= self.prev_x - 0.4 and self.x <= self.prev_x + 0.4: # x close
            if self.y >= self.prev_y - 0.4 and self.y <= self.prev_y + 0.4: # y close
         
                print(self.point)

                # close to waypoint [prev] -> move to waypoint [next]
                msg_p = PoseStamped()
                msg_p.header.frame_id = 'map'
            
                msg_p.header.stamp.secs = self.secs
                msg_p.header.stamp.nsecs = self.nsecs
                msg_p.pose.position.x = self.goal_list[self.point][0] # waypoint [1]..[2]
                msg_p.pose.position.y = self.goal_list[self.point][1]
                q = quaternion_from_euler(0, 0, self.goal_list[self.point][2])
                msg_p.pose.position.z = 0
                msg_p.pose.orientation.z = q[2]
                msg_p.pose.orientation.w = q[3]
         
         #if self.point == 1:
            #print("check speed")
            #rospy.set_param('/move_base/TrajectoryPlannerROS/max_vel_x',0.3)            
            #print("get_param:", rospy.get_param('/move_base/TrajectoryPlannerROS/max_vel_x'))
            
            #client = dynamic_reconfigure.client.Client("/move_base/TrajectoryPlannerROS/set_parameters", timeout=4, config_callback=None)
            #client.update_configuration({"max_vel_x":0.2})

            if self.point == 8:
                rospy.sleep(2)
   
                for i in range(1,3):
                
                    print("send msg")
                    self.goal_pub.publish(msg_p)
                    rospy.sleep(1)

                self.point = self.point + 1

            else :
                pass
        else:
            pass
   

if __name__=="__main__":
    rospy.init_node('test')
    t = test()
    rospy.spin()
#!/usr/bin/env python
"""
 * Copyright (C) 2020, Mohamed Abdelkader.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 """
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool
import tf

class SetpointPublisher:
    def __init__(self):

        self.drone_frame_id_ = rospy.get_param('~drone_frame_id', '/fcu')
        self.tag_frame_id_ = rospy.get_param('~tag_frame_id', '/kf_pipe_link')
        self.tags_topic_ = rospy.get_param('~tags_topic', '/cylinder_centroid_pose')
        self.setpoint_topic_ = rospy.get_param('~setpoint_topic', 'setpoint/relative_pos')
        self.setyaw_topic_ = rospy.get_param('~setpoint_topic', 'setpoint/relative_yaw')

        self.tf_listener_ = tf.TransformListener()

        # Desired altitude above the tag, meters
        self.alt_from_tag_ = rospy.get_param('~alt_from_tag', 1.0)
        # Sanity check. alt_from_tag_ should be non-negative. Otherwise tag will not be seen!
        if self.alt_from_tag_ < 0.0 :
            rospy.logerr("Desired altitude above tag is negative. It should be positive. Defaulting to 1.0 meter")
            self.alt_from_tag_ = 1.0

        # Relative setpoint publisher
        self.setpoint_pub_ = rospy.Publisher(self.setpoint_topic_, Point, queue_size=10)
        self.setyaw_pub_ = rospy.Publisher(self.setyaw_topic_,Float32, queue_size=1)

        rospy.Subscriber(self.tags_topic_, PoseStamped, self.tagsCallback)
        rospy.Subscriber('set_alt_from_target',Float32, self.alt_from_target_cb)

    def alt_from_target_cb (self, msg):
        self.alt_from_tag_ = msg.data

    # tags callback
    def tagsCallback(self, msg):
        valid = False
        trans = []
        if msg.data: # make sure detection is valid

            try:
                (trans,quaternion) = self.tf_listener_.lookupTransform(self.drone_frame_id_, self.tag_frame_id_, rospy.Time(0))
                valid = True
            except :
                rospy.logwarn("No valid TF for the required ")
                valid = False

                
        if valid:
            # for debug
            #rospy.loginfo("Tag %s is x=%s , y=%s , z =%s away from the drone", self.tag_id_, trans[0], trans[1], trans[2])
            
            # compute error in drone frame
            # The one we get in trans is x-forward, y-left, z-up
            # the one we should send is x-right, y-forward, z-up
            ex = -trans[1]
            ey = trans[0]
            ez = trans[2] + self.alt_from_tag_
            sp_msg = Point()
            sp_msg.x = ex
            sp_msg.y = ey
            sp_msg.z = ez
            self.setpoint_pub_.publish(sp_msg)

            if abs(ex) < 0.1 and abs(ey) <0.1:
                #publish the yaw error
                (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quaternion)
                yaw_Sp = Float32()
                yaw_Sp.data = yaw
                self.setyaw_pub_.publish(yaw_Sp)
                

        else: # Publish relative setpoint
            sp_msg = Point()
            sp_msg.x = 0.0
            sp_msg.y = 0.0
            #sp_msg.z = self.alt_from_tag_
            sp_msg.z = 0.0
            self.setpoint_pub_.publish(sp_msg)



            
                


if __name__ == '__main__':
    rospy.init_node('tag_setpoint_publisher_node', anonymous=True)

    sp_o = SetpointPublisher()

    rospy.loginfo("tag_setpoint_publisher_node is started")

    rospy.spin()

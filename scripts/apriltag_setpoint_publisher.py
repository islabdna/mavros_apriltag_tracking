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
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, Float32
from pipe_localization.msg import SetpointMask
import tf

class SetpointPublisher:
    def __init__(self):
        #used to control which axis are controlling
        self.x_enabled = True
        self.y_enabled = True
        self.z_enabled = True
        self.yaw_enabled = True
        self.drone_frame_id_ = rospy.get_param('~drone_frame_id', '/fcu')
        self.tag_frame_id_ = rospy.get_param('~tag_frame_id', '/kf_pipe_link')
        self.tags_topic_ = rospy.get_param('~tags_topic', '/kf_pipe')
        self.setpoint_topic_ = rospy.get_param('~setpoint_topic', 'setpoint/relative_pos')

        self.tf_listener_ = tf.TransformListener()

        # Desired altitude above the tag, meters
        self.alt_from_tag_ = rospy.get_param('~alt_from_tag', 1.0)
        # Sanity check. alt_from_tag_ should be non-negative. Otherwise tag will not be seen!
        if self.alt_from_tag_ < 0.0 :
            rospy.logerr("Desired altitude above tag is negative. It should be positive. Defaulting to 1.0 meter")
            self.alt_from_tag_ = 1.0

        # Relative setpoint publisher
        self.setpoint_pub_ = rospy.Publisher(self.setpoint_topic_, Point, queue_size=10)
        self.valid_setpoint_pub = rospy.Publisher("valid_set_point", Bool,queue_size=1)

        rospy.Subscriber(self.tags_topic_, Point, self.tagsCallback)
        #rospy.Timer(rospy.Duration(0.1),self.tagsCallback)
        rospy.Subscriber('set_alt_from_target',Float32, self.alt_from_target_cb)
        rospy.Subscriber('setpoint_mask',SetpointMask,self.setpoint_mask_cb)

    def setpoint_mask_cb (self,msg):
        self.x_enabled = msg.x_enabled
        self.y_enabled = msg.y_enabled
        self.z_enabled = msg.z_enabled
        self.yaw_enabled = msg.yaw_enabled

    def alt_from_target_cb (self, msg):
        self.alt_from_tag_ = msg.data

    # tags callback
    def tagsCallback(self,msg):
        # print("kf_pipe recived")
        valid = False
        valid_setpoint = Bool()
        trans = []
        try:
            (trans,_) = self.tf_listener_.lookupTransform(self.drone_frame_id_, self.tag_frame_id_, rospy.Time(0))
            valid = True
        except:
            rospy.logwarn("No valid tag's TF")
            valid = False
       

        valid_setpoint.data = valid
        self.valid_setpoint_pub.publish(valid_setpoint)
        
                
        if valid:
            # for debug
            #rospy.loginfo("Tag %s is x=%s , y=%s , z =%s away from the drone", self.tag_id_, trans[0], trans[1], trans[2])
            
            # Keep error in the regular frame
            # we get from tf: x-forward, y-left, z-up
            if self.x_enabled:
                ex_ = trans[0]
            else:
                ex_ = 0.0
            
            if self.y_enabled:
                ey_ = trans[1]
            else:
                ey_ = 0.0
            if self.z_enabled:
                ez_ = trans[2] + self.alt_from_tag_
            else:
                ez_ = 0.0

            br = tf.TransformBroadcaster()
            # syntax: sendTransform((x,y,z), quaternion, time, child_frame, parent_frame)
            
            orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
            br.sendTransform((ex_, ey_, ez_),
                            orientation,
                            rospy.Time.now(),
                            'relative_pos_setpoint',
                            'fcu')

            # convert error to drone frame that we need to send:
            # x-right, y-forward, z-up
            
            ex = -ey_
            ey = ex_
            ez = ez_

            sp_msg = Point()
            sp_msg.x = ex
            sp_msg.y = ey
            sp_msg.z = ez
            self.setpoint_pub_.publish(sp_msg)
            # print("sending to PI controller")
        else: # Publish relative setpoint
            sp_msg = Point()
            sp_msg.x = 0
            sp_msg.y = 0
            sp_msg.z = 0
            self.setpoint_pub_.publish(sp_msg)



            
                


if __name__ == '__main__':
    rospy.init_node('tag_setpoint_publisher_node', anonymous=True)

    sp_o = SetpointPublisher()

    rospy.loginfo("tag_setpoint_publisher_node is started")

    rospy.spin()
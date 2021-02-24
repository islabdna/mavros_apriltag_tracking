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
from pipe_localization.msg import BoundingBox2D
from std_msgs.msg import Bool
import tf

class SetpointPublisher:
    def __init__(self):

    
        self.tags_topic_ = rospy.get_param('~tags_topic', '/bounding_box')
        self.setpoint_topic_ = rospy.get_param('~setpoint_topic', 'setpoint/relative_pos')

        self.tf_listener_ = tf.TransformListener()


        # Relative setpoint publisher
        self.setpoint_pub_ = rospy.Publisher(self.setpoint_topic_, Point, queue_size=10)

        rospy.Subscriber(self.tags_topic_, BoundingBox2D, self.tagsCallback)

    # tags callback
    def tagsCallback(self, msg):
        
                
        if msg.isValid:

            # compute error in drone frame
            # The one we get in pixels is x-right, y-down (image)
            # the one we should send is x-right, y-forward, z-up
            ex = msg.start_x + (msg.width/2) - 640/2
            ey = - (msg.start_y + (msg.height/2) - 480/2) 
            ez = 0
            sp_msg = Point()
            sp_msg.x = ex
            sp_msg.y = ey
            sp_msg.z = ez
            self.setpoint_pub_.publish(sp_msg)
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
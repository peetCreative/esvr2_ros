#!/usr/bin/env python
# !/usr/bin/env python
import rospy
import pygame
import math
import numpy as np
from pygame.locals import K_ESCAPE, K_w, K_s, K_a, K_d, K_q, K_e
from mediassist3_panda_pivoting.msg import LaparoscopeDOFPose
from mediassist3_panda_pivoting.msg import LaparoscopeDOFBoundaries


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.pose_publisher = rospy.Publisher(
            '/gazebo/laparoscope/joint_controller/target/laparoscope_dof_pose',
            LaparoscopeDOFPose, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.boundaries_subscriber = rospy.Subscriber(
            '/gazebo/laparoscope/joint_controller/laparoscope_dof_boundaries',
            LaparoscopeDOFBoundaries, self.update_boundaries)
        self.pose_subscriber = rospy.Subscriber(
            '/gazebo/laparoscope/joint_controller/current/laparoscope_dof_pose',
            LaparoscopeDOFPose, self.update_pose)

        self.current_pose = LaparoscopeDOFPose()
        self.boundaries = LaparoscopeDOFBoundaries()
        # rospy.Timer(rospy.Duration(0.1), self.checkKeyboard)
        self.rate = rospy.Rate(10)

        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption('Python numbers')
        self.screen.fill((159, 182, 205))

        self.font = pygame.font.Font(None, 17)
        self.direction = ""

        self.camera_tilt = -0.52359

    def update_boundaries(self, boundaries):
        self.boundaries = boundaries

    def update_pose(self, pose):
        self.pose = pose

    def move(self):
        pose = LaparoscopeDOFPose()
        pose.yaw = self.pose.yaw
        pose.pitch = self.pose.pitch
        pose.roll = self.pose.roll
        pose.trans_z = self.pose.trans_z
        if self.direction == "up":
            pose.pitch = pose.pitch + 0.01
        if self.direction == "down":
            pose.pitch = pose.pitch - 0.01
        if self.direction == "left":
            pose.yaw = pose.yaw + 0.01
        if self.direction == "right":
            pose.yaw = pose.yaw - 0.01
        if self.direction == "in":
            pose.trans_z = pose.trans_z + 0.01
        if self.direction == "out":
            pose.trans_z = pose.trans_z - 0.01
        if (self.boundaries.yaw_min < pose.yaw < self.boundaries.yaw_max and
                self.boundaries.pitch_min < pose.pitch < self.boundaries.pitch_max):
            tilt_pitch = pose.pitch + self.camera_tilt
            vec1 = np.array([0, math.cos(tilt_pitch), math.sin(tilt_pitch)])
            vec2 = np.array([math.sin(pose.yaw) * math.sin(tilt_pitch),
                             math.cos(tilt_pitch),
                             math.cos(pose.yaw) * math.sin(tilt_pitch)])
            sign_yaw = 1 if pose.yaw > 0 else -1
            angle = np.dot(vec1, vec2)
            if angle < 1.0:
                pose.roll = sign_yaw * math.acos(angle)
            self.pose_publisher.publish(pose)

    def display(self, str):
        dirtext = self.font.render(self.direction, True, (255, 255, 255), (159, 182, 205))
        dirtextRect = dirtext.get_rect()
        dirtextRect.centerx = self.screen.get_rect().centerx
        dirtextRect.centery = self.screen.get_rect().centery + 10

        str = "pitch:{:.2f} yaw:{:.2f} roll{:.2f}".format(
            self.pose.pitch, self.pose.yaw, self.pose.roll)
        posetext = self.font.render(str, True, (255, 255, 255), (159, 182, 205))
        posetextRect = posetext.get_rect()
        posetextRect.centerx = self.screen.get_rect().centerx
        posetextRect.centery = self.screen.get_rect().centery + 10

        str = "pitch_bound:{:.2f},{:.2f} yaw_bound:{:.2f},{:.2f} roll_bound:{:.2f},{:.2f}".format(
            self.boundaries.pitch_min, self.boundaries.pitch_max,
            self.boundaries.yaw_min, self.boundaries.yaw_max,
            self.boundaries.roll_min, self.boundaries.roll_max)
        boundtext = self.font.render(str, True, (255, 255, 255), (159, 182, 205))
        boundtextRect = boundtext.get_rect()
        boundtextRect.centerx = self.screen.get_rect().centerx
        boundtextRect.centery = self.screen.get_rect().centery + 20

        self.screen.blit(dirtext, dirtextRect)
        self.screen.blit(posetext, posetextRect)
        self.screen.blit(boundtext, boundtextRect)
        pygame.display.update()

    def start(self):
        done = False
        print("asd")
        while not done:
            self.display(
                "{} pitch:{} yaw:{} roll{}\n pitch_bound:{},{} yaw_bound:{},{} roll_bound:{},{}".format(
                    self.direction,
                    self.pose.pitch, self.pose.yaw, self.pose.roll,
                    self.boundaries.pitch_min, self.boundaries.pitch_min,
                    self.boundaries.yaw_min, self.boundaries.yaw_min,
                    self.boundaries.roll_min, self.boundaries.roll_min
                ))

            pygame.event.pump()
            keys = pygame.key.get_pressed()
            if keys[K_ESCAPE]:
                done = True
            if keys[K_w]:
                self.direction = "up"
                self.move()
            if keys[K_s]:
                self.direction = "down"
                self.move()
            if keys[K_a]:
                self.direction = "left"
                self.move()
            if keys[K_d]:
                self.direction = "right"
                self.move()
            if keys[K_q]:
                self.direction = "out"
                self.move()
            if keys[K_e]:
                self.direction = "in"
                self.move()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.start()
    except rospy.ROSInterruptException:
        pass

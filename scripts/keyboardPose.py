#!/usr/bin/env python
import rospy
import pygame
import math
import numpy as np
from pygame.locals import K_ESCAPE, K_w, K_s, K_a, K_d, K_q, K_e, K_y, K_x
import argparse
from mediassist3_panda_pivoting.msg import LaparoscopeDOFPose
from mediassist3_panda_pivoting.msg import LaparoscopeDOFBoundaries


class KeyboardDOFController:

    def __init__(self, topic_name):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('keyboard_dof_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.pose_publisher = rospy.Publisher(
            topic_name,
            LaparoscopeDOFPose, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.boundaries_subscriber = rospy.Subscriber(
            '/gazebo/laparoscope/joint_controller/laparoscope_dof_boundaries',
            LaparoscopeDOFBoundaries, self.update_boundaries)
        self.pose_subscriber = rospy.Subscriber(
            '/gazebo/laparoscope/joint_controller/current/laparoscope_dof_pose',
            LaparoscopeDOFPose, self.update_pose)

        self.pose = LaparoscopeDOFPose()
        self.boundaries = LaparoscopeDOFBoundaries()
        # rospy.Timer(rospy.Duration(0.1), self.checkKeyboard)
        self.rate = rospy.Rate(10)

        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption('Python numbers')
        self.screen.fill((159, 182, 205))

        self.font = pygame.font.Font(None, 30)
        self.direction = ""

        #5mm every step
        self.step_distance = 0.005
        #the laparoscope is 30° tilted
        self.camera_tilt = 0.52359
        self.add_camera_tilt = False
        #focus distance in 10 cm
        self.focus_distance = 0.15

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
        trans_pitch = math.asin(
            (math.sin(math.pi - self.camera_tilt) * pose.trans_z)/
            self.focus_distance)
        axis_length = self.focus_distance
        if pose.trans_z != 0:
            axis_length = math.asin(
                (math.sin(math.pi - self.camera_tilt) * pose.trans_z)/
                math.sin(trans_pitch))
        step_pitch = self.step_distance / axis_length
        step_yaw = self.step_distance / axis_length
        if self.direction == "up":
            pose.pitch = pose.pitch + step_pitch
        if self.direction == "down":
            pose.pitch = pose.pitch - step_pitch
        if self.direction == "left":
            pose.yaw = pose.yaw + step_yaw
        if self.direction == "right":
            pose.yaw = pose.yaw - step_yaw
        if self.direction == "in":
            #TODO: with pitching more distance will be covered
            pose.trans_z = pose.trans_z + self.step_distance
        if self.direction == "out":
            pose.trans_z = pose.trans_z - self.step_distance
        if (self.boundaries.yaw_min < pose.yaw < self.boundaries.yaw_max and
                self.boundaries.pitch_min < pose.pitch < self.boundaries.pitch_max and
                self.boundaries.trans_z_min < pose.trans_z < self.boundaries.trans_z_max):
            if self.pose.trans_z != pose.trans_z:
                trans_pitch_new = math.asin(
                    (math.sin(math.pi - self.camera_tilt) * pose.trans_z)/
                    self.focus_distance)
                pose.pitch = pose.pitch - (trans_pitch_new - trans_pitch)
            tilt_pitch = pose.pitch - (self.camera_tilt if self.add_camera_tilt else 0)
            vec1 = np.array([0, math.cos(tilt_pitch), math.sin(tilt_pitch)])
            vec2 = np.array([math.sin(pose.yaw) * math.sin(tilt_pitch),
                             math.cos(tilt_pitch),
                             math.cos(pose.yaw) * math.sin(tilt_pitch)])
            sign_pitch = 1 if pose.pitch > 0 else -1
            sign_yaw = 1 if pose.yaw < 0 else -1
            z_angle = np.dot(vec1, vec2)
            if z_angle < 1.0:
                pose.roll = sign_yaw * sign_pitch * math.acos(z_angle)
            self.pose_publisher.publish(pose)

    def display(self):
        dirtext = self.font.render(self.direction, True, (255, 255, 255), (159, 182, 205))
        dirtextRect = dirtext.get_rect()
        dirtextRect.centerx = self.screen.get_rect().centerx
        dirtextRect.centery = self.screen.get_rect().centery

        str = "pitch:{:.2f} yaw:{:.2f} roll:{:.2f} trans_z:{:.2f}".format(
            self.pose.pitch, self.pose.yaw, self.pose.roll, self.pose.trans_z)
        posetext = self.font.render(str, True, (255, 255, 255), (159, 182, 205))
        posetextRect = posetext.get_rect()
        posetextRect.centerx = self.screen.get_rect().centerx
        posetextRect.centery = self.screen.get_rect().centery + 20

        str = "pitch_bound:{:.2f},{:.2f} yaw_bound:{:.2f},{:.2f} roll_bound:{:.2f},{:.2f}".format(
            self.boundaries.pitch_min, self.boundaries.pitch_max,
            self.boundaries.yaw_min, self.boundaries.yaw_max,
            self.boundaries.roll_min, self.boundaries.roll_max)
        boundtext = self.font.render(str, True, (255, 255, 255), (159, 182, 205))
        boundtextRect = boundtext.get_rect()
        boundtextRect.centerx = self.screen.get_rect().centerx
        boundtextRect.centery = self.screen.get_rect().centery + 40

        self.screen.blit(dirtext, dirtextRect)
        self.screen.blit(posetext, posetextRect)
        self.screen.blit(boundtext, boundtextRect)
        pygame.display.update()

    def start(self):
        done = False
        while not done:
            self.display()
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
            if keys[K_y]:
                self.add_camera_tilt = True
                self.direction = ""
                self.move()
            if keys[K_x]:
                self.add_camera_tilt = False
                self.direction = ""
                self.move()


if __name__ == '__main__':

    topic_name = '/gazebo/laparoscope/joint_controller/target/laparoscope_dof_pose'
    parser = argparse.ArgumentParser(description='Node to which shows field where you can control DOFPose.')
    parser.add_argument('--topic_name', help='topic name to publish to', default=topic_name)
    args = parser.parse_args()
    try:
        x = KeyboardDOFController(args.topic_name)
        x.start()
    except rospy.ROSInterruptException:
        pass

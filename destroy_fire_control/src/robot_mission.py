import math
import rclpy
from rclpy.node import Node
import time
import yaml
from IR_camera.src.find_point_real import SprayFindPointReal
from destroy_fire_control.src.spray_move_test import Spray2FirePoint

class Mission:

    # define mode name
    WAIT_MODE = -1
    NAVIGATE_MODE = 0
    SPRAY_MODE = 1
    GOHOME_MODE = 2

    # initial mode
    mode = WAIT_MODE
    def __init__(self, node):
        self.node = node
        pass



    def __setMode(self, mode):

        if mode not in [
            self.WAIT_MODE,
            self.NAVIGATE_MODE,
            self.SPRAY_MODE,
            self.GOHOME_MODE
        ]:
            self.node.get_logger().error("not a valid mode")
            return False
        self.mode = mode

    def stopMission(self):
        self.__setMode(self.WAIT_MODE)
        self.controller.setZeroVelocity()

    # ---------------------------------------------------------------------------- #
    #                                    Mission                                   #
    # ---------------------------------------------------------------------------- #
    #範例:
    # def templateMission(self):
    #     # 檢查先前模式是否為等待模式，定且設定目前模式為導航模式
    #     if self.mode != self.WAIT_MODE:
    #         self.stopMission()
    #         return False
    #     self.__setMode(self.TEMPLATE_MODE)
    #     # --------------------------------- variable --------------------------------- #
    #     # ----------------------------------- 開始任務 ----------------------------------- #
    #     if self.mode != self.TEMPLATE_MODE:
    #         self.node.get_logger().info("It's not in template mode")
    #         self.stopMission()
    #         return False
    #     # ----------------------------------- 結束任務 ----------------------------------- #
    #     self.stopMission()
    #     return True

    def navigateMission(self):
        # 檢查先前模式是否為等待模式，定且設定目前模式為導航模式
        if self.mode != self.WAIT_MODE:
            self.stopMission()
            return False
        self.__setMode(self.NAVIGATE_MODE)
        # ----------------------------------- 開始任務 ----------------------------------- #
        if self.mode != self.NAVIGATE_MODE:
            self.node.get_logger().info("It's not in navigate mode")
            self.stopMission()
            return False
        # ----------------------------------- 結束任務 ----------------------------------- #
        self.stopMission()
        return True
    
    def sprayMission(self):
        # 檢查先前模式是否為等待模式，定且設定目前模式為滅火模式
        if self.mode != self.WAIT_MODE:
            self.stopMission()
            return False
        self.__setMode(self.SPRAY_MODE)
        # ----------------------------------- 開始任務 ----------------------------------- #
        if self.mode != self.SPRAY_MODE:
            self.node.get_logger().info("It's not in spray mode")
            self.stopMission()
            return False
        
        spray_find_point_real = SprayFindPointReal()
        angle , distance , direction = spray_find_point_real.main()
        self.node.get_logger().info(f"熱源位置: 角度={angle:.2f}°, 距離={distance:.2f}m, 位於熱像儀{direction}")
        spray_2_fire_point = Spray2FirePoint()
        spray_2_fire_point.main(angle, distance)
        # ----------------------------------- 結束任務 ----------------------------------- #
        self.stopMission()
        return True
    
    def gohomeMission(self):
        # 檢查先前模式是否為等待模式，定且設定目前模式為返航模式
        if self.mode != self.WAIT_MODE:
            self.stopMission()
            return False
        self.__setMode(self.GOHOME_MODE)
        # ----------------------------------- 開始任務 ----------------------------------- #
        if self.mode != self.GOHOME_MODE:
            self.node.get_logger().info("It's not in gohome mode")
            self.stopMission()
            return False
        # ----------------------------------- 結束任務 ----------------------------------- #
        self.stopMission()
        return True
    

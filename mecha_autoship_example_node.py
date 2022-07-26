#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus, PointCloud
from mecha_autoship_interfaces.srv import Battery, Actuator, Color

import math


def map(x, input_min, input_max, output_min, output_max):
    res = (x - input_min) * (output_max - output_min) / (
        input_max - input_min
    ) + output_min
    return int(output_min if res < output_min else res)




class MechaAutoshipExampleNode(Node):
    def __init__(self):
        super().__init__("mecha_autoship_example_node")
        self.get_logger().info("mecha_autoship_example_node Start")

        self.data = {
            "IMU": Imu(),
            "GPS": NavSatFix(),
            "LiDAR": PointCloud(),
        }

        # 센서 데이터 Subscribe
        self.imu_sub_handler = self.create_subscription(
            Imu, "imu/data", self.imu_sub_callback, 10
        )
        self.gps_sub_handler = self.create_subscription(
            NavSatFix, "gps/data", self.gps_sub_callback, 10
        )
        self.lidar_sub_handler = self.create_subscription(
            PointCloud, "scan_points", self.lidar_sub_callback, 10
        )

        # 서비스 Client 생성
        self.actuator_set_handler = self.create_client(Actuator, "set_actuator")
        self.color_set_handler = self.create_client(Color, "set_color")

        # 특정 토픽으로부터 데이터를 가져오는 예시입니다. 연결되는 콜백 함수를 참고하세요.
        self.calculate_imu_data = self.create_timer(
            20, self.calculate_imu_data_callback
        )
        self.risk_calculator_lidar_data = self.create_timer(
            1, self.risk_calculator_lidar_data_callback
        )

        # 모터의 쓰로틀을 100%, 각도를 100도로 설정하는 예시입니다.
        self.set_motors(100, 100)
        time.sleep(3)
        self.set_motors(0, 90)  # 기본 상태로 복귀

        # RGB LED의 색상을 빨간색으로 변경하는 예시입니다.
        self.set_pixel(255, 0, 0)
        time.sleep(3)
        self.set_pixel(100, 100, 100)  # 기본 상태로 복귀

    def imu_sub_callback(self, data):
        self.data["IMU"] = data

    def gps_sub_callback(self, data):
        self.data["GPS"] = data

    def lidar_sub_callback(self, data):
        self.data["LiDAR"] = data

    def calculate_imu_data_callback(self):
        self.get_logger().info("Send imu data example")
        # 출력되는 데이터는 쿼터니언으로, 오일러각으로 변환해 응용할 수 있습니다.
        dqx = self.data["IMU"].orientation.x
        dqy = self.data["IMU"].orientation.y
        dqz = self.data["IMU"].orientation.z
        dqw = self.data["IMU"].orientation.w
        
        norm = math.sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz)
        dqw = dqw / norm
        dqx = dqx / norm
        dqy = dqy / norm
        dqz = dqz / norm
        ysqr = dqy * dqy
        
        t3 = 2.0 * (dqw * dqz + dqx * dqy)
        t4 = 1.0 -2.0 * (ysqr + dqz * dqz)
        yaw_raw = math.atan2(t3,t4)
        yaw = yaw_raw * 180.0 / 3.141592
        
        if(yaw > 0):
            yaw = 360 - yaw
        else:
            yaw = abs(yaw)
			
        self.get_logger().info(
            "\nstamp: {0}".format(
                yaw
            )
        )        
    

    def set_motors(self, bldc_pwr, servo_deg):
        """
        쓰로틀과 키의 속도를 설정합니다.
        :param bldc_pwr: 쓰로틀의 속도. 0~100의 범위를 가지고 있습니다.
        :param servo_deg: 키의 각도. 0~180의 입력 범위를 가집니다.
        """
        bldc_pwr = 0 if bldc_pwr < 0 else bldc_pwr
        bldc_pwr = 100 if bldc_pwr > 100 else bldc_pwr
        bldc_pwr = map(bldc_pwr, 0, 100, 100, 140)

        servo_deg = 0 if servo_deg < 0 else servo_deg
        servo_deg = 180 if servo_deg > 180 else servo_deg
        servo_deg = int(servo_deg)

        data_actuator = Actuator.Request()
        data_actuator.throttle_pwr = bldc_pwr
        data_actuator.key_dgr = servo_deg
        self.actuator_set_handler.call_async(data_actuator)
    
    def risk_calculator_lidar_data_callback(self):
        self.get_logger().info("Send lidar data example")
        x_group = []
        y_group = []
        angle_group = []
        risk_group = []
        x_sum = 0
        y_sum = 0
        x_mean = 0
        y_mean = 0
        cnt = 0
        DISTANCE_MIN = 1.0
        # angle 180 ~ -180
        # angle_increment = 0.011 angle_max = 3.15 angle_min = - 3.15
        for data_single in self.data["LiDAR"].points :
            x_group.append(data_single.x)
            y_group.append(data_single.y)
            angle_group.append(math.atan2(data_single.y,data_single.x))
        
         
        risk_group = [0] * len(x_group)    
        
        for i in range(len(x_group)-1):
            if(x_group[i] <= 0):
                risk_group[i] += 10
                
            dist = math.sqrt(math.pow(x_group[i],2) + math.pow(y_group[i],2))  
            if(dist < DISTANCE_MIN):
                risk_group[i] += 1
            if(angle_group[i] * (180/math.pi) > 85 or angle_group[i] * (180/math.pi) < -85):
                risk_group[i] += 1    
             
                 
        angle_and_cnt = []         
        for i in range(len(risk_group)-1):
            if(risk_group[i] == 0):
                x_sum += x_group[i]
                y_sum += y_group[i]
                cnt += 1
            else:
                if(x_sum != 0 and cnt != 0):
                    x_mean = x_sum/cnt	
                    y_mean = y_sum/cnt
                    angle_mean = math.atan2(y_mean,x_mean)
                    angle_and_cnt.append([angle_mean * (180/math.pi),cnt])
                x_sum = 0
                y_sum = 0
                cnt = 0  
                
        Max = 0
        optimal_angle = 0  
        for i in range(len(angle_and_cnt)):
            if(angle_and_cnt[i][1] > Max):
                MAX = angle_and_cnt
                optimal_angle = angle_and_cnt[i][0]      
        self.get_logger().info(
            "\nangle: {0}".format(
                optimal_angle
            )
        )
        if(optimal_angle > 40):
            optimal_angle = 40
        else if(optimal angle < -40):
			optimal_angle = -40
        self.set_motors(70,optimal_angle)    

    def set_pixel(self, _r, _g, _b):
        """
        네오픽셀 링의 R, G, B 값을 설정합니다. 만약 세 값이 동일하다면 RGB는 비활성화되며 흰색 LED가 설정한 밝기로작동합니다.
        :param r: 0~254 범위를 가지는 빨간색 데이터
        :param g: 0~254 범위를 가지는 초록색 데이터
        :param b: 0~254 범위를 가지는 파란색 데이터
        """
        _r = 0 if _r < 0 else _r
        _r = 254 if _r > 254 else _r
        _r = int(_r)

        _g = 0 if _g < 0 else _g
        _g = 254 if _g > 254 else _g
        _g = int(_g)

        _b = 0 if _b < 0 else _b
        _b = 254 if _b > 254 else _b
        _b = int(_b)

        data_rgb = Color.Request()
        data_rgb.red = _r
        data_rgb.green = _g
        data_rgb.blue = _b
        self.color_set_handler.call_async(data_rgb)


def main(args=None):
    rclpy.init(args=args)

    mecha_autoship_example_node = MechaAutoshipExampleNode()

    rclpy.spin(mecha_autoship_example_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

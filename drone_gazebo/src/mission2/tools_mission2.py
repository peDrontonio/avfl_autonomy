#!/usr/bin/python3
import math
import os
import sys
import time

# Add mission2 package to path when running as installed executable
script_dir = os.path.dirname(os.path.abspath(__file__))
mission2_dir = os.path.join(script_dir, 'mission2')
if os.path.exists(mission2_dir) and mission2_dir not in sys.path:
    sys.path.insert(0, mission2_dir)

import numpy as np
import rclpy
from rclpy.node import Node
from states import Search_Base, GoingToBase, Takeoff
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Int32
from drone_navigate.srv import Navigate, GetTelemetry, SetYawRate, NavigateGlobal
from std_srvs.srv import Trigger, SetBool
from ardupilot_msgs.srv import ModeSwitch, ArmMotors, Takeoff as TakeoffSrv
from drone_navigate.msg import BaseDetection
import colorful as cf
from collections import deque
from rclpy.callback_groups import ReentrantCallbackGroup

class Tools(Node):
    def __init__(self) -> None:
        super().__init__('tools_mission2_node')

        # Callback groups for MultiThreadedExecutor
        self.service_cb_group = ReentrantCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        self.fsm = Takeoff()
        self.mission_running = False
        
        # Safety: search attempt counters for RTL
        self.search_attempts = 0
        self.scan_attempts = 0
        self.max_search_attempts = 3  # Máximo de tentativas antes de RTL
        
        # YOLO detection parameters
        self.base_detected = False
        self.detection_confidence = 0.0
        self.x_center = 0
        self.y_center = 0
        self.bbox_width = 0
        self.bbox_height = 0
        self.last_base_coordinates = None
        
        # Consecutive detection tracking
        self.consecutive_detections = 0
        self.required_consecutive_detections = 3
        
        # Image parameters
        self.image_width = 640
        self.image_height = 480
        
        # Focos da camera
        self.fx = 867.5579087775083
        self.fy = 868.4321850250884 

        # Camera service client
        self.activate_camera = self.create_client(SetBool, '/start_camera')
        if self.activate_camera.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Camera service available, trying to activate...')
            req = SetBool.Request()
            req.data = False
            future = self.activate_camera.call_async(req)
        else:
            self.get_logger().info('Camera service not available, continuing anyway')

        # Base GPS coordinates (lat, lon, altitude)
        self.declare_parameter('base_lat', 0.0)
        self.declare_parameter('base_lon', 0.0)
        self.declare_parameter('base_alt', 10.0)  # meters above ground
        
        self.base_lat = self.get_parameter('base_lat').get_parameter_value().double_value
        self.base_lon = self.get_parameter('base_lon').get_parameter_value().double_value
        self.base_alt = self.get_parameter('base_alt').get_parameter_value().double_value
        
        self.get_logger().info(f"Base coordinates - Lat: {self.base_lat}, Lon: {self.base_lon}, Alt: {self.base_alt}m")
        
    def setSubscribers(self):
        self.create_subscription(BaseDetection, '/yolo/base_detection', self.base_detection_callback, 10)
    def setPublishers(self):
        pass
        
    def setClients(self):
        self.navigate = self.create_client(Navigate, 'avfl/navigate', callback_group=self.client_cb_group)
        self.navigate_global = self.create_client(NavigateGlobal, 'avfl/navigate_global', callback_group=self.client_cb_group)
        self.get_telemetry = self.create_client(GetTelemetry, 'avfl/get_telemetry', callback_group=self.client_cb_group)
        self.land_client = self.create_client(ModeSwitch, '/ap/mode_switch', callback_group=self.client_cb_group)
        self.mode_switch_client = self.create_client(ModeSwitch, '/ap/mode_switch', callback_group=self.client_cb_group)
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors', callback_group=self.client_cb_group)
        self.takeoff_client = self.create_client(TakeoffSrv, '/ap/experimental/takeoff', callback_group=self.client_cb_group)        
        

    def setServer(self):
        self.create_service(Trigger, '/start_mission2', self.start_mission_callback, callback_group=self.service_cb_group)

    def wait_for_future(self, future, timeout_sec=10.0):
        """Wait for a future to complete without blocking the executor."""
        start = time.time()
        while not future.done() and (time.time() - start) < timeout_sec:
            time.sleep(0.05)
        return future.done()

    def start_mission_callback(self, request, response):
        self.mission_running = True
        
        # Initialize drone position
        self.get_logger().info("Initializing Mission 2 - Mobile Base Landing")
        
        # Loop da missão
        while self.mission_running and rclpy.ok():
            self.update()
            time.sleep(0.1)
            
        response.success = True
        response.message = "Missão 2 completada!"
        return response

    def base_detection_callback(self, msg):
        '''Callback for YOLO base detection messages'''
        self.base_detected = msg.detected
        if msg.detected:
            self.detection_confidence = msg.confidence
            self.x_center = msg.x_center
            self.y_center = msg.y_center
            self.bbox_width = msg.bbox_width
            self.bbox_height = msg.bbox_height
            
            # Track consecutive detections
            self.consecutive_detections += 1
        else:
            self.consecutive_detections = 0

    def is_base_centered(self, tolerance = 30):
        '''Check if the detected base is centered in the camera frame'''
        if not self.base_detected:
            return False
            
        image_center_x = self.image_width / 2
        image_center_y = self.image_height / 2
        
        offset_x = abs(self.x_center - image_center_x)
        offset_y = abs(self.y_center - image_center_y)
        # print("DEBUG - Offsets: ")
        # print(offset_x, offset_y)
        return offset_x <= tolerance and offset_y <= tolerance 
    
    def is_base_in_x_Axis(self, tolerance = 10):
        if not self.base_detected:
            return False
            
        image_center_x = self.image_width / 2
        
        offset_x = abs(self.x_center - image_center_x)
        # print("DEBUG - Offsets: ")
        # print(offset_x, offset_y)
        return offset_x <= tolerance 

    def is_base_in_y_Axis(self, tolerance = 10):
        if not self.base_detected:
            return False
            
        image_center_y = self.image_height / 2
        
        offset_y = abs(self.y_center - image_center_y)
        # print("DEBUG - Offsets: ")
        # print(offset_x, offset_y)
        return offset_y <= tolerance 

    def return_base(self):
        '''Activate RTL (Return to Base) mode - safety measure'''
        self.get_logger().warn("⚠" * 30)
        self.get_logger().warn("SAFETY: Ativando Return Base!")
        self.get_logger().warn("⚠" * 30)
        req = ModeSwitch.Request()
        req.mode = 6  # RTL mode
        future = self.land_client.call_async(req)
        if self.wait_for_future(future, timeout_sec=5.0):
            result = future.result()
            if result.status:
                self.get_logger().info("RTL mode activated successfully")
                return True
            else:
                self.get_logger().error("Failed to activate RTL mode")
                return False
        else:
            self.get_logger().error("RTL service call timeout")
            return False

    def land(self):
        '''Land the drone by switching to LAND mode'''
        req = ModeSwitch.Request()
        req.mode = 9  # LAND mode
        future = self.land_client.call_async(req)
        if self.wait_for_future(future, timeout_sec=5.0):
            result = future.result()
            if result.status:
                self.get_logger().info("Landing mode activated")
            else:
                self.get_logger().error("Failed to activate landing mode")
        else:
            self.get_logger().error("Landing service call timeout")
    
    def wait_for_landing(self, timeout=120.0):
        '''Wait until the drone has actually landed based only on altitude stability'''
        self.get_logger().info("Aguardando confirmação de pouso (estabilidade de altitude)...")
        
        start_time = time.time()
        
        # Variáveis de controle
        stable_start_time = None
        reference_altitude = 0.0
        
        # Parâmetros de calibração
        landing_region = 1.0      # Altura (m) para começar a monitorar
        stability_margin = 0.10   # Margem (m) permitida de variação em relação à referência
        required_time = 5.0       # Tempo (s) que deve ficar dentro da margem
        
        while rclpy.ok():
            # Timeout de segurança da função
            if time.time() - start_time > timeout:
                self.get_logger().warn(f"Timeout de pouso ({timeout}s)")
                return False
            
            # Chama o serviço de telemetria
            telem_req = GetTelemetry.Request()
            telem_req.frame_id = 'map'
            telem_future = self.get_telemetry.call_async(telem_req)
            
            if self.wait_for_future(telem_future, timeout_sec=1.0):
                telem = telem_future.result()
                current_z = telem.z
                
                # Se a altitude for maior que 1.0m, o drone ainda está voando alto
                if current_z > landing_region:
                    if stable_start_time is not None:
                        self.get_logger().info(f"Altitude subiu ({current_z:.2f}m). Resetando timer.")
                        stable_start_time = None
                    
                    self.get_logger().info(f"Descendo... Alt: {current_z:.2f}m", throttle_duration_sec=2.0)
                
                # Entrou na região de pouso (< 1.0m)
                else:
                    if stable_start_time is None:
                        # Começa a contar o tempo agora.
                        # TRAVA a altitude atual como referência.
                        stable_start_time = time.time()
                        reference_altitude = current_z
                        self.get_logger().info(f"Zona de pouso alcançada ({current_z:.2f}m). Iniciando verificação de estabilidade...")
                    
                    else:
                        # Compara a altitude atual com a REFERÊNCIA travada (não com a anterior)
                        diff = abs(current_z - reference_altitude)
                        
                        if diff > stability_margin:
                            # Se a diferença for grande, significa que o drone ainda está descendo (mesmo que devagar)
                            # Atualizamos a referência para a nova posição e zeramos o timer
                            self.get_logger().info(f"Variação detectada ({diff:.3f}m). Ainda em movimento...")
                            reference_altitude = current_z
                            stable_start_time = time.time()
                        
                        elif (time.time() - stable_start_time) > required_time:
                            # Se passou 5 segundos e a altitude nunca saiu da margem de 0.1m da referência
                            self.get_logger().info(f"Pouso confirmado! Altitude estável em {reference_altitude:.2f}m por {required_time}s.")
                            return True
            
            time.sleep(0.2)
        
        return False

    def takeoffWait(self, z, auto_arm=True):
        '''Arm motors and takeoff to specified altitude'''
        self.get_logger().info(f"Starting takeoff sequence to {z}m altitude...")
                # Step 0: Switch to GUIDED mode
        self.get_logger().info("Switching to GUIDED mode...")
        mode_req = ModeSwitch.Request()
        mode_req.mode = 4  # GUIDED mode
        mode_future = self.mode_switch_client.call_async(mode_req)
        
        if self.wait_for_future(mode_future, timeout_sec=5.0):
            mode_result = mode_future.result()
            if mode_result.status:
                self.get_logger().info("GUIDED mode activated")
            else:
                self.get_logger().error("Failed to switch to GUIDED mode")
                return False
        else:
            self.get_logger().error("Mode switch service call timeout")
            return False
                # Step 1: Arm the motors
        if auto_arm:
            self.get_logger().info("Arming motors...")
            arm_req = ArmMotors.Request()
            arm_req.arm = True
            arm_future = self.arm_client.call_async(arm_req)
            
            if self.wait_for_future(arm_future, timeout_sec=5.0):
                arm_result = arm_future.result()
                if arm_result.result:
                    self.get_logger().info("Motors armed successfully!")
                else:
                    self.get_logger().error("Failed to arm motors")
                    return False
            else:
                self.get_logger().error("Arm motors service call timeout")
                return False
        
        # Step 2: Takeoff to specified altitude
        self.get_logger().info(f"Taking off to {z}m...")
        takeoff_req = TakeoffSrv.Request()
        takeoff_req.alt = float(z)
        takeoff_future = self.takeoff_client.call_async(takeoff_req)
        
        if self.wait_for_future(takeoff_future, timeout_sec=5.0):
            takeoff_result = takeoff_future.result()
            # Note: ArduPilot Takeoff service might not have a 'result' field
            # Just check if service call completed
            self.get_logger().info("Takeoff command sent, waiting for altitude...")
            
            # Wait until we reach the target altitude
            start_time = time.time()
            timeout = 30.0  # 30 seconds timeout
            
            while rclpy.ok() and (time.time() - start_time) < timeout:
                telem_req = GetTelemetry.Request()
                telem_req.frame_id = 'map'
                telem_future = self.get_telemetry.call_async(telem_req)
                
                if self.wait_for_future(telem_future, timeout_sec=1.0):
                    telem = telem_future.result()
                    current_alt = telem.z
                    
                    self.get_logger().info(f"Current altitude: {current_alt:.2f}m / Target: {z:.2f}m", 
                                         throttle_duration_sec=1.0)
                    
                    # Check if we've reached target altitude (within tolerance)
                    if abs(current_alt - z) < 0.5:
                        self.get_logger().info(f"Target altitude reached: {current_alt:.2f}m")
                        return True
                
                time.sleep(0.2)
            
            self.get_logger().error("Timeout waiting for target altitude")
            return False
        else:
            self.get_logger().error("Takeoff service call timeout")
            return False

    def navigateGlobalWait(self, lat, lon, z, yaw=float('nan'), speed=0.5, tolerance=0.2, auto_arm=True):
        '''Navigate to GPS coordinates and wait until target is reached'''
        try:
            # Wait for service to be available
            if not self.navigate_global.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("Navigate global service not available")
                return None
            
            req = NavigateGlobal.Request()
            req.lat = float(lat)
            req.lon = float(lon)
            req.z = float(z)
            req.yaw = float(yaw)
            req.yaw_rate = 0.0
            req.speed = float(speed)
            req.frame_id = 'map'
            req.auto_arm = auto_arm
            
            self.get_logger().info(f"Navigating to GPS: Lat={lat:.6f}, Lon={lon:.6f}, Alt={z}m")
            
            future = self.navigate_global.call_async(req)
            
            # Wait for service response (MultiThreadedExecutor handles callbacks)
            if not self.wait_for_future(future, timeout_sec=30.0):
                self.get_logger().error("Global navigation service call timeout after 30 seconds")
                return None
            
            res = future.result()
            if not res.success:
                self.get_logger().error(f"Failed to start global navigation: {res.message}")
                return res
            
            self.get_logger().info("Global navigation started, waiting to reach target...")
            
            # Wait for TF propagation and drone to start moving
            time.sleep(2.0)
            
            # Safety timeout: allow max 120 seconds for GPS navigation
            nav_start = time.time()
            nav_timeout = 120.0
            
            # Wait until we reach the target
            while rclpy.ok():
                if time.time() - nav_start > nav_timeout:
                    self.get_logger().warn(f"navigateGlobalWait timeout ({nav_timeout}s)")
                    return res
                
                telem_req = GetTelemetry.Request()
                telem_req.frame_id = 'navigate_target'
                telem_future = self.get_telemetry.call_async(telem_req)
                
                if not self.wait_for_future(telem_future, timeout_sec=1.0):
                    continue
                
                telem = telem_future.result()
                distance = math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2)
                
                self.get_logger().info(f"Distance to GPS target: {distance:.2f}m (tolerance: {tolerance}m)", throttle_duration_sec=2.0)
                
                if distance < tolerance:
                    self.get_logger().info("Target GPS location reached!")
                    return res
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Error during global navigation: {e}")
            return None

    def navigateWait(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.2, frame_id='map', tolerance=0.1, auto_arm=True, z_participation = 1):
        '''Navigate without interruption, wait until target is reached '''
        try:
            req = Navigate.Request()
            req.x = float(x)
            req.y = float(y)
            req.z = float(z)
            req.yaw = float(yaw)
            req.yaw_rate = 0.0
            req.speed = float(speed)
            req.frame_id = frame_id
            req.auto_arm = auto_arm
            
            future = self.navigate.call_async(req)
            
            if not self.wait_for_future(future, timeout_sec=10.0):
                self.get_logger().error("Navigação timeout")
                return None
            
            res = future.result()
            if not res.success:
                self.get_logger().error("Falha na navegação")
                return res
            self.get_logger().info("Navegação iniciada")
            
            # Safety timeout
            nav_distance = math.sqrt(float(x)**2 + float(y)**2 + float(z)**2)
            nav_timeout = max(10.0, (nav_distance / max(speed, 0.1)) * 3.0 + 5.0)
            nav_start = time.time()
            
            while rclpy.ok():
                if time.time() - nav_start > nav_timeout:
                    self.get_logger().warn(f"navigateWait timeout ({nav_timeout:.1f}s)")
                    return res
                
                telem_req = GetTelemetry.Request()
                telem_req.frame_id = 'navigate_target'
                telem_future = self.get_telemetry.call_async(telem_req)
                
                if not self.wait_for_future(telem_future, timeout_sec=1.0):
                    continue
                
                telem = telem_future.result()
                distance = math.sqrt(telem.x ** 2 + telem.y ** 2 + (telem.z * z_participation) ** 2)
                # self.get_logger().info(f"Distância até o alvo: {distance:.2f} m")
                if distance < tolerance:
                    self.get_logger().info("Alvo alcançado")
                    return res
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Erro ao chamar o serviço de navegação: {e}")

    def navigateInterrupted(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.2, frame_id='map', tolerance=0.1, auto_arm=False, z_participation = 1, center_tolerance = 30):
        '''Navigate without interruption, wait until target is reached '''
        tentativas_maximas = 2
        for i in range(tentativas_maximas):  # Limita o número de tentativas
            try:
                req = Navigate.Request()
                req.x = float(x)
                req.y = float(y)
                req.z = float(z)
                req.yaw = float(yaw)
                req.yaw_rate = 0.0
                req.speed = float(speed)
                req.frame_id = frame_id
                req.auto_arm = auto_arm
                
                future = self.navigate.call_async(req)
                
                if not self.wait_for_future(future, timeout_sec=10.0):
                    self.get_logger().error("Navegação timeout")
                    return "failed"
                
                res = future.result()
                if not res.success:
                    self.get_logger().error("Falha na navegação")
                    return "failed"
                self.get_logger().info("Navegação iniciada")
                while rclpy.ok():
                    telem_req = GetTelemetry.Request()
                    telem_req.frame_id = 'navigate_target'
                    telem_future = self.get_telemetry.call_async(telem_req)
                    
                    if not self.wait_for_future(telem_future, timeout_sec=1.0):
                        continue
                    
                    telem = telem_future.result()
                    
                    if self.consecutive_detections > self.required_consecutive_detections:
                        map_req = GetTelemetry.Request()
                        map_req.frame_id = 'map'
                        map_future = self.get_telemetry.call_async(map_req)
                        if self.wait_for_future(map_future, timeout_sec=1.0):
                            self.last_base_coordinates = map_future.result()
                            self.last_base_coordinates.x = self.last_base_coordinates.x - (self.y_center - self.image_height//2)/self.fy * self.last_base_coordinates.z
                            self.last_base_coordinates.y = self.last_base_coordinates.y - (self.x_center - self.image_width//2)/self.fx * self.last_base_coordinates.z
                    
                    distance = math.sqrt(telem.x ** 2 + telem.y ** 2 + (telem.z * z_participation) ** 2)
                    # self.get_logger().info(f"Distância até o alvo: {distance:.2f} m")
                    if (distance < tolerance):
                        self.get_logger().info("Base não encontrada nessa direção. Tentando novamente.")
                        y = -2 * y # Inverte a direção e dobra a distância
                        auto_arm = False
                        break
                    if self.is_base_centered(center_tolerance):
                        self.get_logger().info("Base encontrada e centrada. Alvo alcançado.")
                        self.navigateWait(x=0, y = 0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                        return "success"
            except Exception as e:
                self.get_logger().error(f"Erro ao chamar o serviço de navegação: {e}")
                
        self.get_logger().info(f"A base não foi centralizada corretamente. Procurar novamente...")
        return "failed"

    def navigateCentralize(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=False, z_participation = 1, center_tolerance = 10):
        '''Navigate without interruption, wait until target is reached '''
        try:
            res = self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
            if not res.success:
                self.get_logger().error("Falha na navegação")
                return res
            self.get_logger().info("Navegação iniciada")
            while rclpy.ok():
                telem = self.get_telemetry(frame_id='navigate_target')
                if self.consecutive_detections > self.required_consecutive_detections:
                    self.last_base_coordinates = self.get_telemetry(frame_id='map')
                    self.last_base_coordinates.x = self.last_base_coordinates.x - (self.y_center - self.image_height//2)/self.fy * self.last_base_coordinates.z
                    self.last_base_coordinates.y = self.last_base_coordinates.y - (self.x_center - self.image_width//2)/self.fx * self.last_base_coordinates.z
                distance = math.sqrt(telem.x ** 2 + telem.y ** 2 + (telem.z * z_participation) ** 2)
                # self.get_logger().info(f"Distância até o alvo: {distance:.2f} m")
                if (distance < tolerance):
                    self.get_logger().info("Limite de movimento alcançado.")
                    return "failure"
                if self.is_base_in_x_Axis(center_tolerance):
                    self.get_logger().info("Base encontrada e centrada. Alvo alcançado.")
                    self.navigateWait(x=0, y = 0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                    return "success"
        except Exception as e:
            self.get_logger().error(f"Erro ao chamar o serviço de navegação: {e}")
                
        self.get_logger().info(f"A base não foi centralizada corretamente. Procurar novamente...")
        return "failed"
        
    # def navigate_with_compensation(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=True):
    #     '''Navigate with lead compensation for moving base'''
    #     try:
    #         # Calculate lead position based on base velocity
    #         # Assuming 0.5 m/s base velocity and some processing delay
    #         processing_delay = 0.5  # seconds
    #         lead_distance = self.base_velocity * processing_delay
            
    #         # Compensate x position (assuming base moves in x direction)
    #         compensated_x = x + lead_distance
            
    #         res = self.navigate(x=compensated_x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    #         if not res.success:
    #             self.get_logger().error("Falha na navegação com compensação")
    #             return res
    #         self.get_logger().info("Navegação com compensação iniciada")
    #         return res
    #     except Exception as e:
    #         self.get_logger().error(f"Erro ao chamar o serviço de navegação: {e}")


    def navigateWaitTeste(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=True, z_participation = 1):
        '''Navigate without interruption, wait until target is reached '''
        try:
            self.get_logger().info("Navegação iniciada")
            inicial = self.get_telemetry(frame_id='map')
            if frame_id == "body":
                inicial.x += x
                inicial.y += y
                inicial.z += z
            if frame_id == "map":
                inicial.x = x
                inicial.y = y
                inicial.z = z
            while rclpy.ok():
                diferenca = self.get_telemetry(frame_id='map')
                diferenca.x, diferenca.y, diferenca.z = (inicial.x - diferenca.x), (inicial.y - diferenca.y), (inicial.z - diferenca.z)
                distance = math.sqrt(diferenca.x ** 2 + diferenca.y ** 2 + (diferenca.z * z_participation) ** 2)
                # self.get_logger().info(f"Distância até o alvo: {distance:.2f} m")
                # self.get_logger().info(f"X: {diferenca.x}\nY: {diferenca.y}\n Z: {diferenca.z}\n")
                if distance < tolerance:
                    self.get_logger().info("Alvo alcançado")
                    return (0, 'Sucesso')
        except Exception as e:
            self.get_logger().error(f"Erro ao chamar o serviço de navegação: {e}")


    def navigateInterruptedTeste(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=False, z_participation = 1, center_tolerance = 30):
        '''Navigate without interruption, wait until target is reached '''
        tentativas_maximas = 2
        for i in range(tentativas_maximas):  # Limita o número de tentativas
            try:
                self.get_logger().info("Navegação iniciada")
                inicial = self.get_telemetry(frame_id='map')
                if frame_id == "body":
                    inicial.x += x
                    inicial.y += y
                    inicial.z += z
                if frame_id == "map":
                    inicial.x = x
                    inicial.y = y
                    inicial.z = z
                while rclpy.ok():
                    diferenca = self.get_telemetry(frame_id='map')
                    diferenca.x, diferenca.y, diferenca.z = (inicial.x - diferenca.x), (inicial.y - diferenca.y), (inicial.z - diferenca.z)
                    distance = math.sqrt(diferenca.x ** 2 + diferenca.y ** 2 + (diferenca.z * z_participation) ** 2)
                    if self.consecutive_detections > self.required_consecutive_detections:
                        self.last_base_coordinates = self.get_telemetry(frame_id='map')
                        self.last_base_coordinates.x = self.last_base_coordinates.x - (self.y_center - self.image_height//2)/self.fy * self.last_base_coordinates.z
                        self.last_base_coordinates.y = self.last_base_coordinates.y - (self.x_center - self.image_width//2)/self.fx * self.last_base_coordinates.z
                    # self.get_logger().info(f"Distância até o alvo: {distance:.2f} m")
                    # self.get_logger().info(f"X: {diferenca.x}\nY: {diferenca.y}\n Z: {diferenca.z}\n")
                    if (distance < tolerance):
                        self.get_logger().info("Base não encontrada nessa direção. Tentando novamente.")
                        y = -2 * y # Inverte a direção e dobra a distância
                        auto_arm = False
                        break
                    if self.is_base_centered(center_tolerance):
                        self.get_logger().info("Base encontrada e centrada. Alvo alcançado.")
                        self.navigateWait(x=0, y = 0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                        return "success"
            except Exception as e:
                self.get_logger().error(f"Erro ao chamar o serviço de navegação: {e}")
                
        self.get_logger().info(f"A base não foi centralizada corretamente. Procurar novamente...")
        return "failed"

    def navigateCentralizeTeste(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=False, z_participation = 1, center_tolerance = 10):
        '''Navigate without interruption, wait until target is reached '''
        try:
            self.get_logger().info("Navegação iniciada")
            inicial = self.get_telemetry(frame_id='map')
            if frame_id == "body":
                inicial.x += x
                inicial.y += y
                inicial.z += z
            if frame_id == "map":
                inicial.x = x
                inicial.y = y
                inicial.z = z
            while rclpy.ok():
                diferenca = self.get_telemetry(frame_id='map')
                diferenca.x, diferenca.y, diferenca.z = (inicial.x - diferenca.x), (inicial.y - diferenca.y), (inicial.z - diferenca.z)
                distance = math.sqrt(diferenca.x ** 2 + diferenca.y ** 2 + (diferenca.z * z_participation) ** 2)
                if self.consecutive_detections > self.required_consecutive_detections:
                    self.last_base_coordinates = self.get_telemetry(frame_id='map')
                    self.last_base_coordinates.x = self.last_base_coordinates.x - (self.y_center - self.image_height//2)/self.fy * self.last_base_coordinates.z
                    self.last_base_coordinates.y = self.last_base_coordinates.y - (self.x_center - self.image_width//2)/self.fx * self.last_base_coordinates.z
                distance = math.sqrt(diferenca.x ** 2 + diferenca.y ** 2 + (diferenca.z * z_participation) ** 2)
                self.get_logger().info(f"Distância até o alvo: {distance:.2f} m")
                if (distance < tolerance):
                    self.get_logger().info("Limite de movimento alcançado.")
                    return "failure"
                if self.is_base_in_x_Axis(center_tolerance):
                    self.get_logger().info("Base encontrada e centrada. Alvo alcançado.")
                    self.navigateWait(x=0, y = 0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                    return "success"
        except Exception as e:
            self.get_logger().error(f"Erro ao chamar o serviço de navegação: {e}")
                
        self.get_logger().info(f"A base não foi centralizada corretamente. Procurar novamente...")
        return "failed"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # === НАСТРОЙКА ARUCO ДЕТЕКТОРА ===
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        
        # === ПАРАМЕТРЫ ДЛЯ РАСЧЕТА РАССТОЯНИЯ ===
        self.MARKER_REAL_WIDTH = 0.17
        self.focal_length = 1000
        
        # === НАСТРОЙКА КАМЕРЫ ===
        self.cap = None
        
        # Пробуем открыть камеру
        for camera_index in [0, 1]:
            try:
                self.cap = cv2.VideoCapture(camera_index)
                if self.cap.isOpened():
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    self.cap.set(cv2.CAP_PROP_FPS, 30)
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    
                    ret, frame = self.cap.read()
                    if ret:
                        self.get_logger().info(f"Camera {camera_index} opened successfully")
                        break
                    else:
                        self.cap.release()
                        self.cap = None
            except:
                if self.cap:
                    self.cap.release()
                    self.cap = None
        
        if self.cap is None:
            self.get_logger().error("No camera found!")
            return
            
        self.get_logger().info("ArUco Detector started")
        self.timer = self.create_timer(0.05, self.process_frame)

    def calculate_marker_size_and_distance(self, corners):
        """Расчет размера и расстояния до маркера"""
        try:
            if corners is None or len(corners) == 0:
                return 0, 0, 0
                
            marker_corners = corners[0]
            top_left = marker_corners[0][0]
            top_right = marker_corners[0][1] 
            bottom_left = marker_corners[0][3]
            
            # Вычисляем ширину и высоту
            width_pixels = np.linalg.norm(top_right - top_left)
            height_pixels = np.linalg.norm(bottom_left - top_left)
            
            if width_pixels > 0:
                distance = (self.MARKER_REAL_WIDTH * self.focal_length) / width_pixels
                return width_pixels, height_pixels, distance
            return 0, 0, 0
                
        except:
            return 0, 0, 0

    def process_frame(self):
        if self.cap is None:
            return
            
        ret, frame = self.cap.read()
        if not ret:
            return
        
        try:
            # Детектирование маркеров
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict)
            
            # Обработка найденных маркеров
            if ids is not None:
                frame = aruco.drawDetectedMarkers(frame, corners, ids)
                
                for i, marker_id in enumerate(ids):
                    marker_id = int(marker_id[0])
                    
                    # Вычисляем центр маркера
                    center_x = int(np.mean(corners[i][0][:, 0]))
                    center_y = int(np.mean(corners[i][0][:, 1]))
                    
                    # Расчет размера и расстояния
                    width_px, height_px, distance = self.calculate_marker_size_and_distance([corners[i]])
                    
                    # Отображение информации о маркере
                    if distance > 0:
                        # ID маркера (зеленый, жирный)
                        cv2.putText(frame, f"ID: {marker_id}", 
                                   (center_x - 25, center_y - 55),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        # Надпись "Distance:" (фиолетовый, тонкий)
                        cv2.putText(frame, "Distance:", 
                                   (center_x - 35, center_y - 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)  # Фиолетовый
                        
                        # Значение расстояния (фиолетовый, тонкий)
                        cv2.putText(frame, f"{distance:.2f} m", 
                                   (center_x - 25, center_y - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)  # Фиолетовый
                        
                        # Размеры в пикселях (светло-желтый, тонкий)
                        cv2.putText(frame, f"Size: {width_px:.0f}x{height_px:.0f} px", 
                                   (center_x - 35, center_y + 15),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    
                    # Центральная точка
                    cv2.circle(frame, (center_x, center_y), 6, (0, 0, 255), -1)
            
            # Простой информационный текст
            cv2.putText(frame, "ArUco Detector", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            if ids is None:
                cv2.putText(frame, "No markers", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                cv2.putText(frame, f"Markers: {len(ids)}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Отображение кадра
            cv2.imshow('ArUco Detector', frame)
            
            # Выход по Q
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.destroy_node()
                
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {str(e)}")

    def destroy_node(self):
        self.get_logger().info("Shutting down ArUco Detector...")
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

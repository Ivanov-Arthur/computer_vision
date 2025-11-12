#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Инициализация детектора ArUco (старый API для совместимости)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        
        # Инициализация камеры с проверкой разных индексов
        self.cap = None
        for camera_index in [0, 1, 2, 3]:
            self.cap = cv2.VideoCapture(camera_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # Увеличиваем разрешение
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
            # Проверяем, открылась ли камера
            ret, test_frame = self.cap.read()
            if ret and test_frame is not None:
                self.get_logger().info(f"Camera found at index {camera_index}")
                break
            else:
                self.cap.release()
                self.cap = None
        
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error("No camera found! Please check camera connection.")
            return
            
        self.get_logger().info("ArUco Detector started!")
        self.timer = self.create_timer(0.03, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to grab frame")
            return
        
        try:
            # Конвертируем в grayscale для лучшего детектирования
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Детектирование маркеров (старый API)
            corners, ids, rejected = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
            
            # Отображение результатов
            if ids is not None:
                # Рисуем обнаруженные маркеры
                frame = aruco.drawDetectedMarkers(frame, corners, ids)
                
                # Выводим ID маркеров
                for i, marker_id in enumerate(ids):
                    marker_id = int(marker_id[0])
                    
                    # Вычисляем центр маркера для текста
                    center_x = int(np.mean(corners[i][0][:, 0]))
                    center_y = int(np.mean(corners[i][0][:, 1]))
                    
                    # Отображаем ID маркера
                    cv2.putText(frame, f"ID: {marker_id}", 
                               (center_x - 20, center_y - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    
                    # Рисуем точку в центре маркера
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                    
                    self.get_logger().info(f"Detected ArUco marker ID: {marker_id}", 
                                         throttle_duration_sec=1.0)
            
            # Добавляем информационный текст
            cv2.putText(frame, "ArUco Detector - Press 'q' to quit", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            if ids is None:
                cv2.putText(frame, "No markers detected", 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(frame, f"Markers found: {len(ids)}", 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Увеличиваем размер окна
            cv2.namedWindow('ArUco Detector', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('ArUco Detector', 1280, 720)  # Большое окно
            
            # Показываем frame
            cv2.imshow('ArUco Detector', frame)
            
            # Выход по нажатию 'q'
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.destroy_node()
            elif key == ord('r'):  # Reset detection
                self.get_logger().info("Manual reset")
                
        except Exception as e:
            self.get_logger().error(f"Error in frame processing: {str(e)}")

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

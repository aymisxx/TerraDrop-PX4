#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

class DualCylinderEstimator(Node):
    def __init__(self):
        super().__init__('dual_cylinder_estimator')
        
        qos = rclpy.qos.QoSProfile(depth=1)
        self.depth_sub = self.create_subscription(
            Image, '/drone/front_depth', self.depth_callback, qos)
        self.bridge = CvBridge()
        self.get_logger().info("Dual Cylinder Estimator initialized. Press 'q' in the overlay window to finish and display final estimates.")
        
        # Lists for storing valid measurements (each measurement is a dict with time, width, height, depth)
        self.cylinder1_measurements = []
        self.cylinder2_measurements = []
        
        self.start_time = None
        
        # Pinhole camera parameter (focal length in pixels) – adjust as needed.
        self.focal_length = 525.0
        
        # Valid depth range (in meters). Only use pixels with depth between these values.
        self.valid_depth_min = 0.1
        self.valid_depth_max = 20.0
        
        # Minimum bounding box size (in pixels) to consider a candidate.
        self.min_bbox_width = 30
        self.min_bbox_height = 30
        
        # Optionally, ignore boxes that cover too much of the frame.
        self.max_bbox_width_ratio = 0.7  # no more than 70% of image width
        self.max_bbox_height_ratio = 0.7 # no more than 70% of image height
        
        # Set up live matplotlib plotting for width and height over time.
        plt.ion()
        self.fig, self.axs = plt.subplots(2, 1, figsize=(8, 6))
        self.axs[0].set_title("Cylinder Width (m) Over Time")
        self.axs[0].set_xlabel("Time (s)")
        self.axs[0].set_ylabel("Width (m)")
        self.axs[1].set_title("Cylinder Height (m) Over Time")
        self.axs[1].set_xlabel("Time (s)")
        self.axs[1].set_ylabel("Height (m)")
        self.plot_timer = self.create_timer(1.0, self.plot_callback)
        
        # OpenCV window for live overlay.
        cv2.namedWindow("Cylinder Estimation Overlay", cv2.WINDOW_NORMAL)
    
    def depth_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
        current_time = time.time() - self.start_time
        
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error("Error converting depth image: " + str(e))
            return
        
        # Replace NaNs with 0.
        depth_image = np.nan_to_num(depth_image, nan=0.0)
        
        # Prepare an overlay image (normalize for display).
        disp_img = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        disp_img = disp_img.astype(np.uint8)
        disp_img = cv2.cvtColor(disp_img, cv2.COLOR_GRAY2BGR)
        
        # Preprocess for detection.
        blurred = cv2.medianBlur(depth_image.astype(np.float32), 5)
        norm_img = cv2.normalize(blurred, None, 0, 255, cv2.NORM_MINMAX)
        norm_img = norm_img.astype(np.uint8)
        edges = cv2.Canny(norm_img, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Get image dimensions.
        img_h, img_w = depth_image.shape[:2]
        
        # Filter out small or excessively large bounding boxes.
        candidates = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if (w >= self.min_bbox_width and h >= self.min_bbox_height and
                w < self.max_bbox_width_ratio * img_w and h < self.max_bbox_height_ratio * img_h and
                cv2.contourArea(cnt) > 1000):
                candidates.append(cnt)
        
        # Sort candidates left-to-right.
        candidates = sorted(candidates, key=lambda cnt: cv2.boundingRect(cnt)[0])
        
        # Determine assignments: if at least 2 candidates are found, use the leftmost for Cylinder 1 and rightmost for Cylinder 2.
        # If only one candidate is detected, assign it based on its center relative to the image center.
        cand1 = None
        cand2 = None
        if len(candidates) >= 2:
            cand1 = candidates[0]
            cand2 = candidates[-1]
        elif len(candidates) == 1:
            x, y, w, h = cv2.boundingRect(candidates[0])
            center_x = x + w / 2.0
            if center_x < (img_w / 2.0):
                cand1 = candidates[0]
            else:
                cand2 = candidates[0]
        
        # Process candidate for Cylinder 1.
        if cand1 is not None:
            meas1 = self.process_candidate(cand1, depth_image, disp_img, current_time, label="C1", color=(0,255,0))
            if meas1 is not None:
                self.cylinder1_measurements.append(meas1)
        # Process candidate for Cylinder 2.
        if cand2 is not None:
            meas2 = self.process_candidate(cand2, depth_image, disp_img, current_time, label="C2", color=(0,0,255))
            if meas2 is not None:
                self.cylinder2_measurements.append(meas2)
        
        cv2.imshow("Cylinder Estimation Overlay", disp_img)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.shutdown_node()
    
    def process_candidate(self, cnt, depth_image, overlay, current_time, label="C1", color=(0,255,0)):
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(overlay, (x, y), (x+w, y+h), color, 2)
        roi = depth_image[y:y+h, x:x+w]
        # Only consider pixels with depth between valid_depth_min and valid_depth_max.
        valid_pixels = roi[(roi >= self.valid_depth_min) & (roi <= self.valid_depth_max)]
        if valid_pixels.size == 0:
            return None
        median_depth = float(np.median(valid_pixels))
        width_m = (w * median_depth) / self.focal_length
        height_m = (h * median_depth) / self.focal_length
        text = f"{label}: D={median_depth:.2f}m, W={width_m:.2f}m, H={height_m:.2f}m"
        cv2.putText(overlay, text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return {"time": current_time, "width": width_m, "height": height_m, "depth": median_depth}
    
    def plot_callback(self):
        self.axs[0].clear()
        self.axs[1].clear()
        if self.cylinder1_measurements:
            times1 = [m["time"] for m in self.cylinder1_measurements]
            widths1 = [m["width"] for m in self.cylinder1_measurements]
            heights1 = [m["height"] for m in self.cylinder1_measurements]
            self.axs[0].plot(times1, widths1, 'g.-', label="Cylinder 1")
            self.axs[1].plot(times1, heights1, 'g.-', label="Cylinder 1")
        if self.cylinder2_measurements:
            times2 = [m["time"] for m in self.cylinder2_measurements]
            widths2 = [m["width"] for m in self.cylinder2_measurements]
            heights2 = [m["height"] for m in self.cylinder2_measurements]
            self.axs[0].plot(times2, widths2, 'r.-', label="Cylinder 2")
            self.axs[1].plot(times2, heights2, 'r.-', label="Cylinder 2")
        self.axs[0].set_title("Cylinder Width (m) Over Time")
        self.axs[0].set_xlabel("Time (s)")
        self.axs[0].set_ylabel("Width (m)")
        self.axs[0].legend()
        self.axs[1].set_title("Cylinder Height (m) Over Time")
        self.axs[1].set_xlabel("Time (s)")
        self.axs[1].set_ylabel("Height (m)")
        self.axs[1].legend()
        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)
    
    def shutdown_node(self):
        self.get_logger().info("Shutting down and computing final estimates...")
        if self.cylinder1_measurements:
            widths1 = [m["width"] for m in self.cylinder1_measurements]
            heights1 = [m["height"] for m in self.cylinder1_measurements]
            final_width1 = np.median(widths1)
            final_height1 = np.median(heights1)
            self.get_logger().info(f"Final Cylinder 1 - Diameter: {final_width1:.2f} m, Height: {final_height1:.2f} m")
        else:
            self.get_logger().info("No valid measurements for Cylinder 1.")
        if self.cylinder2_measurements:
            widths2 = [m["width"] for m in self.cylinder2_measurements]
            heights2 = [m["height"] for m in self.cylinder2_measurements]
            final_width2 = np.median(widths2)
            final_height2 = np.median(heights2)
            self.get_logger().info(f"Final Cylinder 2 - Diameter: {final_width2:.2f} m, Height: {final_height2:.2f} m")
        else:
            self.get_logger().info("No valid measurements for Cylinder 2.")
        
        rclpy.shutdown()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = DualCylinderEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Dual Cylinder Estimator interrupted.")
    finally:
        if rclpy.ok():
            node.shutdown_node()
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
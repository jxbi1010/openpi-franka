import pyrealsense2 as rs
import numpy as np
import time 

class RealSenseCamera:
    """RealSense camera pipeline manager."""
    
    def __init__(
        self,
        serial_number: str,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ):
        """
        Initialize RealSense camera pipeline.
        
        Args:
            serial_number: Serial number of the RealSense camera
            width: Image width in pixels
            height: Image height in pixels
            fps: Frames per second
        """
        self.serial_number = serial_number
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.config = None
        self.start()
        
    def start(self):
        """Start the RealSense camera pipeline."""
        # Create pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure pipeline
        self.config.enable_device(self.serial_number)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        
        # Start streaming
        self.pipeline.start(self.config)

        print(f"RealSense camera {self.serial_number} started: {self.width}x{self.height} @ {self.fps}fps")
        
    def stop(self):
        """Stop the RealSense camera pipeline."""
        if self.pipeline:
            self.pipeline.stop()
            print(f"RealSense camera {self.serial_number} stopped")
            
    def get_image(self) -> np.ndarray:
        """
        Get RGB image from RealSense camera.
        
        Returns:
            RGB image as numpy array, shape (H, W, 3), dtype uint8
        """            
        # Wait for frames
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())  # Shape: (H, W, 3), dtype: uint8, BGR format
        
        # Convert BGR to RGB
        rgb_image = color_image[..., ::-1]  # Shape: (H, W, 3), dtype: uint8, RGB format
        
        return rgb_image
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()


if __name__ == "__main__":
    import cv2

    wrist_camera = RealSenseCamera(serial_number="254522070421", width=640, height=480, fps=30)
    exterior_camera = RealSenseCamera(serial_number="254522070220", width=640, height=480, fps=30)
    with wrist_camera, exterior_camera:
        image = wrist_camera.get_image()
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow("Wrist Image", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        image = exterior_camera.get_image()
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow("Exterior Image", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

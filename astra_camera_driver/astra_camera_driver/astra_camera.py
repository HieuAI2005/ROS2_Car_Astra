import cv2
import numpy as np
from openni import openni2
from openni import _openni2 as c_api

class Camera:
    def __init__(self, 
                 fps=30, 
                 width=640, 
                 height=480, 
                 openni_libs='libs/'):
        #config default parameters.
        self.fps = fps
        self.width = width
        self.height = height
        self.openni_libs = openni_libs
        self.wait_time = int(1000.0/float(fps))
        self.load()
    
    def unload(self):
        openni2.unload()
        
    def load(self):
        openni2.initialize('/home/hiwe/ros2_ws/src/astra_camera_driver/libs')
        self.dev = openni2.Device.open_any()

        # init depth stream
        if not self.dev.has_sensor(openni2.SENSOR_DEPTH):
            raise RuntimeError("Device does not have depth sensor")

        self.depth_stream = self.dev.create_depth_stream()
        self.depth_stream.set_video_mode(
            c_api.OniVideoMode(
                pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM,
                resolutionX=self.width,
                resolutionY=self.height,
                fps=self.fps,
            )
        )
        self.depth_stream.start()

        # init color stream
        if not self.dev.has_sensor(openni2.SENSOR_COLOR):
            raise RuntimeError("Device does not have color sensor")

        self.color_stream = self.dev.create_color_stream()
        self.color_stream.set_video_mode(
            c_api.OniVideoMode(
                pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888,
                resolutionX=self.width,
                resolutionY=self.height,
                fps=self.fps,
            )
        )
        self.color_stream.start()

        # enable registration
        self.dev.set_image_registration_mode(
            c_api.OniImageRegistrationMode.ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR
        )

        
    def get_depth(self):
        frame = self.depth_stream.read_frame()
        frame_data = frame.get_buffer_as_uint16()
        # Put the depth frame into a numpy array and reshape it
        img = np.frombuffer(frame_data, dtype=np.uint16)
        img.shape = (1, 480, 640)
        img = np.concatenate((img, img, img), axis=0)
        img = np.swapaxes(img, 0, 2)
        img = np.swapaxes(img, 0, 1)
        return img
    
    def get_color(self):
        frame = self.color_stream.read_frame()
        frame_data = frame.get_buffer_as_uint8()
        colorPix = np.frombuffer(frame_data, dtype=np.uint8)
        colorPix.shape = (self.height, self.width, 3)
        return colorPix
    
    def get_depth_and_color(self):
        return self.get_depth(), self.get_color()
    

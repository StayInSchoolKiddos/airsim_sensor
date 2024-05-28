import airsim 
import numpy as np
import os 
import pprint
import time as time 
"""
This is a toy script to test the connection to AirSim and get the camera info
Trying to figure out how the api works
https://microsoft.github.io/AirSim/image_apis/
"""

pp = pprint.PrettyPrinter(indent=4)

#get WSL_HOST_IP
WSL_HOST_IP = os.getenv('WSL_HOST_IP', 'localhost')
print('WSL_HOST_IP: ', WSL_HOST_IP)

client = airsim.VehicleClient(
    ip=WSL_HOST_IP
)
client.confirmConnection()

## api to access camera data
print('Connected to AirSim')
front_camera = 'front_center_custom'
camera_info = client.simGetCameraInfo(front_camera)
print("CameraInfo:")
pp.pprint(camera_info)

#api to get image and display
responses = client.simGetImages([
    airsim.ImageRequest(front_camera, airsim.ImageType.Scene, False, False)
])
image = responses[0]

# api to process image
print("Image:")
print(type(image))
start_time = time.time()
# get numpy array
img1d = np.frombuffer(image.image_data_uint8, dtype=np.uint8) 
# reshape array to 4 channel image array H X W X 4
img_rgb = img1d.reshape(image.height, image.width, 3)
print("time taken to reshape: ", time.time() - start_time)
# original image is fliped vertically
# img_rgb = np.flipud(img_rgb)
filename = 'airsim_camera_image'
# write to png 
airsim.write_png(os.path.normpath(filename + '.png'), img_rgb) 
end_time = time.time()
print('Image saved as ' + filename + '.png')
print('Time taken: ', end_time - start_time)
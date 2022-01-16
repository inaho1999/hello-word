# coding=utf-8
import math
# distance to object (mm) = focal length (mm) * real height of the object (mm) * image height (pixels)
#                           ----------------------------------------------------------------
#                                 object height (pixels) * sensor height (mm)
#focal_length_mm = 3.37
height_of_camera_bottom = 0.458744257689
height_of_camera_top = 0.50361686945
ball_radius_reality = 0.05
ball_radius_image_0 = 20
distance_0 = math.sqrt(height_of_camera_top**2+1.59**2)
focal_length_0 = ball_radius_image_0*distance_0/ball_radius_reality
print('focal_legnth của camera 0: {0}'.format(focal_length_0))
distance_1 = math.sqrt(height_of_camera_bottom**2+0.8**2)
ball_radius_image_1 = 49
focal_length_1 = ball_radius_image_1*distance_1/ball_radius_reality
print('focal_legnth của camera 1: {0}'.format(focal_length_1))
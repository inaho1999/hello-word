# coding=utf-8
import math
# distance to object (mm) = focal length (mm) * real height of the object (mm) * image height (pixels)
#                           ----------------------------------------------------------------
#                                 object height (pixels) * sensor height (mm)
#focal_length_mm = 3.37
height_of_camera_bottom = 0.442734718323
height_of_camera_top = 0.50361686945
ball_radius_reality = 0.05
ball_radius_image_0 = 20
distance_0 = math.sqrt(height_of_camera_top**2+1.59**2)
focal_length_0 = ball_radius_image_0*distance_0/ball_radius_reality
print('focal_legnth của camera 0: {0}'.format(focal_length_0))
distance_1 = math.sqrt(height_of_camera_bottom**2+0.25**2)
ball_radius_image_1 = 83
focal_length_1 = ball_radius_image_1*distance_1/ball_radius_reality
print('focal_legnth của camera 1: {0}'.format(focal_length_1))


# height_real_camera_1 = 0.4671172797679901
# radius_pixel_1 = 29.5
# radius_real = 0.052 /2
# distance_to_ball_real = 0.41
# distance_to_camera = math.sqrt(height_real_camera_1**2 + distance_to_ball_real**2)
# focal_length_1 = radius_pixel_1 *distance_to_camera / radius_real
# print('focal length 1:{0}'.format(focal_length_1))
# # 465.192307692

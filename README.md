# Fast-BGsub-object-localisation

## Method and pipeline
![crow_pipeline_straight](https://user-images.githubusercontent.com/44549945/145674812-503e26cd-9588-4374-a826-7e3a06beda69.png)
Raw image in HD stream is published as a ROS message by the intel realsense camera node which is. Using CV_Bridge ROS message is converted into readable cv2 arrays for further preprocessing.

As soon as the program is started, we assume that the workspace is empty, the background is captured and saved. This saved empty background is used for background subtraction to segment out the newly introduced objects in the workspace. We keep a threshold value to tackle noise, shadow and lighting variations. 

Once we perform background subtraction, we get the segmentation of the newly introduced parts in the robot workspace. Using edge detection techniques, various objects are localised and cropped images of these small objects are cut out which reduces the dimensionality by factors of 100. 

These cropped frames are then pushed for object detection to predict their names or class labels. The master subscribes the object detection results are published by the slaves and incooperate the visual scene for planning, collision avoidance and human-collaborative tasks in real-time,

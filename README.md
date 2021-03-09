# Yolo-Cutter_ROS
Basic implementation of YOLOv3 algorithm with an image cutter for the ROI, fully automated by ROS.

This test mini-project is part of the RUTAS project:

![image](https://user-images.githubusercontent.com/80078826/110398940-e69a4980-8042-11eb-8c4c-8092b7a55a60.png)


You must have installed ROS, the Pepper Environment (you can relate to this GitHub: https://github.com/manoelpla/scriptpepper) and any Python IDE (here I use PyCharm) to run everything. It is extremely recommended to run everything in Python2.7. It can also be run with any other robot, as long as it has a /camera/image_raw/ and /camera/image_raw/compressed in its published topics.

We need to install YOLOv3 (as its done in this GitHub: https://github.com/leggedrobotics/darknet_ros.git):

    $sudo apt-get install -y gcc-6                                                # Avoid having Anaconda installed to make sure you don't have errors.
  
    $mkdir -p catkin_ws/src                                                       # This is for placing YOLO in the catkin_ws
  
    $cd ~/catkin_ws/src/
  
    $git clone --recursive https://github.com/leggedrobotics/darknet_ros.git      # Install YOLOv3.
  
    $cd ~/catkin_ws/
  
    $catkin build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-6    # Build YOLOv3 in Ubuntu18.04LTS.
  
          $catkin build -DCMAKE_BUILD_TYPE=Release                              # Run this in case you have Ubuntu16.04.
  
    $gedit ~/.bashrc                                                              # Modify bashrc, for example: source /home/alejandro/catkin_ws/devel/setup.bash
  
    $reboot
  
In the file 'ros.yaml' stored in ~/catkin_ws/src/darknet_ros/darknet_ros/config/

Change camera_reading topic from “/camera/rgb/image_raw” to “/pepper/camera/front/image_raw” to finish the setup of the YOLOv3 and leave it ready.

After installing everything, you can have an environment just like this one:

![image](https://user-images.githubusercontent.com/80078826/110397632-34fa1900-8040-11eb-8306-eb4f7a172518.png)

If you run the environment and YOLOv3:

    $roslaunch pepper_gazebo_plugin pepper_gazebo_plugin_in_office_CPU.launch
    $roslaunch darknet_ros darknet_ros.launch

You'll have something like this:

![image](https://user-images.githubusercontent.com/80078826/110398059-0cbeea00-8041-11eb-8b1f-6d79af14a0c0.png)

The interesting part comes here, for everything to be done automatically, we need to subscribe to two topics that are being published when both simulations starts: 

1. /pepper/camera/front/image_raw/compressed -> this is to get the visual information of what's receiving the 2D frontal camera of the Pepper.

That is done with this line in the GCut_ROS.py program:

    rospy.Subscriber('/pepper/camera/front/image_raw/compressed', CompressedImage, callback, queue_size=5)

Here, the callback is the magical part, because it's programmed to do all the hard part of converting the image information that we are receiving from the topic into an actual image to be processed. We must take into account that all the data that ROS published goes in some sort of string-numbers-matrix form and must be assembled as an image for processing.

    np_arr = np.fromstring(ros_data.data, np.uint8)
    
In the np_arr line, we are converting the data received in from the topic into a Numpy Array.

    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
Later, we use the img line to decode the data and turn it into an image, available for processing.

    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback2, queue_size=5)

The second subscriber gets the bounding boxes information from the topic BoundingBoxes, which stores all the info related to the output of YOLOv3. The part that we are interested in is the xmin, ymin, xmax, ymax variables, which are the ones that represent the coordinates of the bounding box (if you are wondering, we would have more coordinates in the case that Pepper sees more than one object).

    for box in bounding_boxes:
        x, y, x1, y1 = box
        roi = img[int(y):int(y1), int(x):int(x1)]

The 'for' loops helps us to navigate through the Bounding_Box tuple and extract the values needed to crop and obtain roi, which is the Region Of Interest (ROI) in the picture. 

    cv2.imshow('img',roi)
    cv2.waitKey(50)
    
Remember to show the img you're processing! It's always a great way to debug your programs to see your outputs and analyze if that's what you're expecting! The cv2.waitKey() is always un ms (milliseconds), if you put 1, you're going to have the little window frozen because your PC won't be able to show that quantity of images in one second. With 50 it's ok to see the output without getting anything frozen.

If you did everything good, you'll start to see something like this:

![image](https://user-images.githubusercontent.com/80078826/110399491-efd7e600-8043-11eb-9691-42f8500720b4.png)

Remember to pay a lot of attention in this lines:

    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', roi)[1]).tostring()
    # Publish new image
    image_pub.publish(msg)

Those lines are the ones in charge to prepare the image for publishing. The 'msg' line defines the type of message ROS is going to send through the publishing topic (in this case is a CompressedImage(), just like the one that has been used for processing). The 'msg.header.stamp' gets and the ROS time of the simulation and adds it as a header to the message. The 'msg.format' declares the format of the message (if you like, you can change it to png or other type). The most important line comes here: the 'msg.data'! This is the one in charge to convert the 'roi' image we cropped into a message available for ROS to send! If we don't convert it, ROS would have problems to send it and a Traceback would appear. The 'img.publish(msg)' invokes the publiher that is declareed below in the code:

    image_pub = rospy.Publisher("Pepper_Publisher", CompressedImage, queue_size=5)

Remember to declare a queue_size in your Publisher and Subscribers! Without one of those your sintax is incomplete and a traceback will appear! The queue_size is the number of elements that ROS can have in a line waiting to be sent, if you have a high quantity of messages, you can have a big queue_size, but if you don't have many, it's better to have a little one (I wouldn't recommend queue_size=1, when I tried it, I had tracebacks).

    rospy.spin()
    
Finally, the rospy.spin() line. This little command is very important! It's the one in charge to keep your node alive. If you don't write it, once the process of the GCut_ROS.py is done, ROS will close the node and you won't be able to see anything being published and if you do rostopic info to any of the topics you're subscribing to in the code, you'll see that Pepper_subscriber is not on the list. That happens because your node has already been closed (rospy.spin() is absent!)
 
And that's it! Now you have a fully connected detector and ROI cropper working on ROS! Thanks a lot for implementing this, feel free to use it as much as you like (but always cite me please!). If you make this better, please let me know with an issue (that also counts with any bugs you find). Also, I would like to thank all the authors of the GitHubs I've used in this work.

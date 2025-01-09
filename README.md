#Introduction：

  The inspection robot proposed in this project, based on ROS (Robot Operating System) and WSN (Wireless Sensor Network), is capable of constructing a map of its surrounding environment and continuously monitoring environmental data. The collected data is then uploaded to a cloud platform. This robot has wide applications in areas such as warehouse logistics and intelligent inspection, with promising prospects in the market.


#Hardware Requirements：

  1.ROS Four-Wheel Robot (equipped with NVIDIA Jetson Nano B01, STM32 microcontroller, SLAM LiDAR RPLIDAR A1, Orbbec Astra S Depth Camera, system USB drive, and Wi-Fi wireless network card, etc.)  
  2.CC2530 ZigBee EndDevice and Coordinator.  
  3.Environmental Sensors: DHT11 Temperature and Humidity Sensor; Gas Sensor (MQ-2); PIR Motion Sensor.
![Hardware Requirements](https://github.com/user-attachments/assets/298e4b3e-45ef-48b7-ad97-4af97d668fa0)


#Software Requirements：

  1.The operating system of the PC (VMware virtual machine) is Ubuntu 18.04.6 LTS.  
  2.The ROS version is Melodic 1.14.13.  
  3.Qt Creator version 4.9.2.  
  4.The system for editing and downloading to the CC2530 development board is Windows 10.  
  5.IAR Embedded Workbench for 8051 V8.10.


#Program Application Setup：

ZigBee Wireless Sensor Network：

  1.Navigate to the directory “ZStack-CC2530-2.5.1a/Projects/zstack/Samples/GenericApp/CC2530DB/” and open the GenericApp.eww project using IAR Embedded Workbench for 8051.  
  2.Build the “EndDevice” and “Coordinator” separately, then use CC Debugger to download the application to the CC2530 board. (Note: When downloading the EndDevice, manually modify the node ID value at line 151 in the enddevice.c file, for example, change it to ‘1’ or ‘2’).  
  3.Connect all sensors to the EndDevice and link the Coordinator to the robot via the serial port.  


Qt-based Remote Monitoring Application：

  1.In the home directory of the Ubuntu system, create a new workspace using the command line and compile it. Download and extract the "qt_ros_test.zip" package file to the src folder of this workspace. Then, set the appropriate permissions for the package and compile it.

  2.Install the required dependencies (e.g., expect) and add the path of the package's script folder to the .bashrc file:export PATH=$PATH:/home/your_directory_name/your_workspace_name/src/qt_ros_test/script. Afterward, initialize the environment variables.

  3.Open the /etc/hosts file and add the robot's IP address and username to this file, then restart the network.

  4.Open the qt_ros_test.launch file and change the virtual machine password to your own password.

  5.Configure the virtual machine with dual network adapters, setting both to bridge mode. One should be connected to the PC’s wireless network card, and the other should be connected to the Ethernet.

  6.After navigating to the directory, enter the “bash initssh.sh”and “roslaunch qt_ros_test qt_ros_test.launch” commands in the terminal to start the package。


Create OneNET Corresponding Device and Data Stream：

  1.Log in to the OneNET cloud platform and register the relevant product (select HTTP as the access protocol). Obtain the product name, ID, device key, etc., and use the token to calculate and obtain the encrypted security authentication key.

  2.Modify the parameters related to your device information in the uploadData() function in the main_window.cpp file of the package mentioned above.

  3.Set the corresponding data stream model in the product section of the OneNET cloud platform.
![Data Stream](https://github.com/user-attachments/assets/9c5965ea-9c5e-4dc5-b5f5-3af1b5a0e8e2)


#Real-time Inspection Status and Environmental Data Viewing：

  After starting the software, click the "connect" button on the right to establish a remote connection with the robot. Once "Online" appears at the top left of the software, the connection is successful. You can enable the camera and keyboard control of the robot by checking the "camera" and "keyboard control" buttons. Additionally, you can quickly launch the Rviz visualization interface, mapping functions, navigation features, etc., through other buttons. Clicking the "Get Node Data" button will retrieve the data from each node in the ZigBee wireless sensor network and display it in real-time on the interface. Using the "send_marker" button, you can send specified coordinate information to Rviz for display. The environmental data collected in the above process will be uploaded via the HTTP protocol to the corresponding device's data stream on the OneNET cloud platform.
![GUI-1](https://github.com/user-attachments/assets/ec6a7814-5743-4e67-9274-766f03d59cac)
![GUI-2](https://github.com/user-attachments/assets/7c7b99b0-4381-45b0-b80b-de9db5418d80)
<img width="1280" alt="Cloud-1" src="https://github.com/user-attachments/assets/541e7abb-1d7b-4e7f-acae-c0df87319367" />
<img width="1280" alt="Cloud-2" src="https://github.com/user-attachments/assets/cbe0ffb0-53c0-4975-b258-d4bafd39b2a3" />

  The related demo video is as follows: 【Online Monitoring Software for Inspection Car Based on ROS and WSN - Bilibili】 https://b23.tv/D4zk1kA


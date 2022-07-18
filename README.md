# SensorFusion
Data fusion solutions for IMU sensors in MATLAB

Run ArduinoQuaternion first to get the biases (sensors have run-to-run biases). Then run StateOfTheArtComparison and see the difference to MATLAB's own imufilter implementation

Hardware config: 

Arduino Nano 3
MPU - 6050 (accel + gyro)

![image](https://user-images.githubusercontent.com/81184255/179507012-5037c3da-6fbf-4166-a7c0-cd539bf88132.png)

![qkf](https://user-images.githubusercontent.com/81184255/179507307-474d4e89-54a0-45c0-b766-2c8e626e5eac.gif)

![image](https://user-images.githubusercontent.com/81184255/179507342-5f3b7f15-19e0-4187-96d0-5f9744cfb9d0.png)

SmartCap: IoT-Enabled Navigation for the Visually Impaired


SmartCap is a wearable assistive technology designed to help blind and visually impaired individuals navigate their surroundings safely. 
By combining ultrasonic spatial sensing with IoT-based emergency alerts, it provides both immediate obstacle detection and a safety net for loved ones.

Key Features:

1) Tri-Directional Sensing: Uses three ultrasonic sensors to monitor the front, left, and right paths.
2) Haptic Feedback: Dual vibration motors provide intuitive tactile alerts (Left motor for left obstacles, Right for right, both for center).
3) Fall Detection System: Integrated MPU6050 Accelerometer/Gyroscope to detect sudden impacts or unusual orientation changes (indicating a potential fall).
4) Emergency SMS: Integrated ESP8266 module connects to IFTTT to send GPS location data to a pre-configured phone number via a trigger and if fall is detected.
5) Low Profile: Built on the Arduino Nano platform for a lightweight, wearable form factor.

Component,Purpose:

Arduino Nano (main microcontroller)

ESP8266 (NodeMCU)

MPU6050 (6-axis Gyroscope & Accelerometer)

3x HC-SR04,Ultrasonic sensors

2x Vibration Motors

2x NPN Transistors BC547

Buzzer,(Optional) For audible low-battery or critical alerts

Power Source,9V Battery or Li-po

Schematic Diagram:


<img width="3176" height="1952" alt="smart caap for visually handicap" src="https://github.com/user-attachments/assets/2e53c65c-eedb-46ef-8891-5d32d833bcb1" />

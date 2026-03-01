# Realtime-sorting-vehicle
A soft real-time autonomous vehicle project based on Raspberry Pi, focusing on distance control, safety, and multi-threaded system design.

Project  overview
This project inplements a multi-threaded autonomous robot system running on Raspberry Pi.
The robot integrates:
1. Line tracking (infrared sensors)
2. Ultrasonic obstacle detection 
3. Real-time motor control
4. Thread synchronization
5. safe shutdown handling (Ctrl+C)

The system is designed using modular architecture and concurrent executionm ensuring smooth coordination between tracking and obstacle avoidance tasks.

System Architecture:
+-------------------+
|     main.cpp      |
|-------------------|
| Thread Management |
| Signal Handling   |
+-------------------+
        |
        ↓
+-------------------+       +----------------------+
| Line Tracker      |       | Obstacle Avoidance   |
| (line_tracker)    |       | (ultrasonic_sensor)  |
+-------------------+       +----------------------+
            ↓                     ↓
                +------------------+
                | Motor Controller |
                +------------------+

Module Description:
1. Line Tracking Module
Functionality:
a): Reads infrared sensors
b): Determines deviation from line
c): Sends direction command to motor controller

Code logic:
void LineTracker::evaluateDirection()
{
	// the current level state is read only when hardware interrupt is triggered
	int leftState = gpioRead(leftpin);
	int centerState = gpioRead(centerpin);
	int rightState = gpioRead(rightpin);

	int direction = 2;	// line lose

	if (centerState == 1) {
		direction = 0;	// center
	}
	else if (leftState == 1) {
		direction = -1;	// left, corrected to the right
	}
	else if (rightState == 1) {
		direction = 1;	// right, corrected to the left
	}

	if (m_callback) {
		m_callback(direction);
	}
}

2. Ultrasonic Sensor Module
Functionality:
a): Sends trigger pulses
b): Measures echo return time 
c): Calculates distance
    distance = times * (speed of sound) / 2

Code logic:
void Ultrasonic_Sensor::handleEchoInterrupt(int level, uint32_t tick)
{
	if (level == 1) {
		startTick = tick;	// record rising edge time
	}
	else if (level == 0) {
		uint32_t diff = tick - startTick;	// calculate the time difference on the falling edge
		float distance = (diff * 0.0343f) / 2.0f;	// calculate distance - cm

		if (m_callback) {
			m_callback(distance);
		}
	}
}

In this part, provides real-time distance data to obstacle avoidance module.

3. Obstacle Avoidance Module
Code logic:
void ObstacleAvoidance::processDistanceUpdate(float currentDistanceCm)
{
	if (currentDistanceCm < threshold) {
		// obstacles detected 
		if (!currentlyBlocked) {
			currentlyBlocked = true;
			std::cout << "[obstacle avoidance system] detects obstacles (" << currentDistanceCm << "cm ), Stop!\n";
			if (m_alertCallback) {
				m_alertCallback(true);	// notify the main system -> stop!
			}
		}
	}
	else {
		// no obstacles
		if (currentlyBlocked) {
			currentlyBlocked = false;
			std::cout << "[obstacle avoidance system] No obstacles! (" << currentDistanceCm << "cm ), Pass!\n";
			if (m_alertCallback) {
				m_alertCallback(false); // notify the main system -> release stop!
			}
		}
	}
}

And uses mutex locking to prevent motor conflict:  
std::mutex motorMutex;

4. Motor controller
Responsibilities:
a): COntrol GPIO pins
b): Set motor direction
c): Adjust speed
d): Prevent concurrent access

This part is protected by:
std::mutex motorMutex;
Ensures only one thread can control motors at a time.

5. Multi-Threading Design
The system uses:
std::atomic<bool> running
std::mutex
std::condition_variable

Threads:
Line Tracking Thread    -    Continuous line following
Obstacle Thread    -    Distance monitoring
Main Thread    -    System coordination

6. Safe Shutdown Handing
Program captures:
signal(SIGINT, signalHandler);
When user presses: "Ctrl + C"

It:
a): Sets running = false
b): Joins all threads
c): Stops motor safely
d): Cleans up pipgio

7. Build Instructions
a): Install pipgio:
sudo apt update
sudo apt install pigpio
sudo systemctl start pigpiod

b): Compile
g++ -std=c++17 main.cpp \
    line_tracker.cpp \
    ultrasonic_sensor.cpp \
    obstacle_avoidance.cpp \
    motor_controller.cpp \
    -lpigpio -lpthread -o robot

c): Run
sudo ./robot

Press "Ctrl + C" to stop

8. Future Improvements
a): PID control for smoother tracking
b): Sensor fusion optimization
c): Dynamic speed adjustment
d): Logging system

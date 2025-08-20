# drone-navigation
An autonomous navigation python app that connects to the vehicle using pynavlink.

The app does the following actions:
1. Mode set to GUIDED
2. Arming...
3. Taking off to 10 meters...
4. Rotating yaw by 90°...
5. Mode set to LAND
6. Landing...

Testing using Ardupilot SITL:
```
python3 sim_vehicle.py -v copter --console --map -w --out=127.0.0.1:14551
```
After SITL or a real Quad is running, run the app:
```
python3 drone_mavlink.py
```
https://github.com/user-attachments/assets/a05326f6-b045-4392-a16a-b541c8f1d328


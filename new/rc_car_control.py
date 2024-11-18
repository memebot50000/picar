import asyncio
import json
import time
from gpiozero import Motor, OutputDevice
import websockets
from aiohttp import web

# Set up the motors
drive_motor = Motor(forward=27, backward=17, enable=12)
steer_motor = Motor(forward=22, backward=23, enable=13)

# Set up the failsafe switch
failsafe = OutputDevice(18)  # Use GPIO 18 for the failsafe switch

# Steering control variables
last_steer_time = 0
last_steer_direction = 0
steer_cooldown = 0.5  # 0.5 seconds cooldown

# Drive motor dead zone
DRIVE_DEAD_ZONE = 0.2  # 20% dead zone

# Armed state
armed = False

def set_motors(drive, steer):
    global last_steer_time, last_steer_direction
    
    if not armed:
        drive_motor.value = 0
        steer_motor.value = 0
        failsafe.off()
        return

    failsafe.on()
    
    # Apply dead zone to drive motor
    if abs(drive) < DRIVE_DEAD_ZONE:
        drive = 0
    
    drive_motor.value = drive
    
    # Steering logic
    current_time = time.time()
    if steer != 0 and (current_time - last_steer_time > steer_cooldown):
        if (steer > 0 and last_steer_direction <= 0) or (steer < 0 and last_steer_direction >= 0):
            steer_value = 1 if steer > 0 else -1
            steer_motor.value = steer_value
            last_steer_time = current_time
            last_steer_direction = steer_value
            # Schedule to stop the steering motor after 0.5 seconds
            asyncio.create_task(stop_steering_after_delay(0.5))
    elif steer == 0:
        last_steer_direction = 0

async def stop_steering_after_delay(delay):
    await asyncio.sleep(delay)
    steer_motor.value = 0

async def websocket_handler(websocket, path):
    global armed
    try:
        async for message in websocket:
            data = json.loads(message)
            if 'armed' in data:
                armed = data['armed']
                if not armed:
                    set_motors(0, 0)
            else:
                drive = data['y']
                steer = data['x']
                set_motors(drive, steer)
    finally:
        set_motors(0, 0)

async def index(request):
    return web.Response(text='''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RC Car Control</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.9.0/nipplejs.min.js"></script>
    <style>
        #joystick {
            width: 200px;
            height: 200px;
            margin: 50px auto;
            border: 1px solid black;
        }
        #armed-switch {
            margin: 20px auto;
            text-align: center;
        }
    </style>
</head>
<body>
    <div id="armed-switch">
        <label for="armed">Armed:</label>
        <input type="checkbox" id="armed" name="armed">
    </div>
    <div id="joystick"></div>
    <script>
        const socket = new WebSocket('ws://' + window.location.hostname + ':8765');
        const armedSwitch = document.getElementById('armed');
        
        const joystick = nipplejs.create({
            zone: document.getElementById('joystick'),
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: 'blue',
            size: 150
        });

        armedSwitch.addEventListener('change', (event) => {
            socket.send(JSON.stringify({ armed: event.target.checked }));
        });

        joystick.on('move', (evt, data) => {
            if (armedSwitch.checked) {
                const x = data.vector.x;
                const y = -data.vector.y;  // Invert Y-axis
                socket.send(JSON.stringify({ x, y }));
            }
        });

        joystick.on('end', () => {
            if (armedSwitch.checked) {
                socket.send(JSON.stringify({ x: 0, y: 0 }));
            }
        });
    </script>
</body>
</html>
    ''', content_type='text/html')

async def main():
    app = web.Application()
    app.router.add_get('/', index)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    
    ws_server = await websockets.serve(websocket_handler, '0.0.0.0', 8765)
    
    print("Server started at http://0.0.0.0:8080")
    await asyncio.Future()  # Run forever

if __name__ == '__main__':
    asyncio.run(main())

import asyncio
import json
from gpiozero import Motor
import websockets
from aiohttp import web

# Set up the motors
drive_motor = Motor(forward=17, backward=27, enable=12)
steer_motor = Motor(forward=22, backward=23, enable=13)

def set_motors(drive, steer):
    drive_motor.value = drive
    steer_motor.value = steer

async def websocket_handler(websocket, path):
    try:
        async for message in websocket:
            data = json.loads(message)
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
    </style>
</head>
<body>
    <div id="joystick"></div>
    <script>
        const socket = new WebSocket('ws://' + window.location.hostname + ':8765');
        
        const joystick = nipplejs.create({
            zone: document.getElementById('joystick'),
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: 'blue',
            size: 150
        });

        joystick.on('move', (evt, data) => {
            const x = data.vector.x;
            const y = -data.vector.y;  // Invert Y-axis
            socket.send(JSON.stringify({ x, y }));
        });

        joystick.on('end', () => {
            socket.send(JSON.stringify({ x: 0, y: 0 }));
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

import asyncio
import json
import websockets
import httpx

async def test_flow():
    async with httpx.AsyncClient() as client:
        # 1. Reset
        await client.post('http://localhost:8000/api/simulation/reset')
        
        # 2. Add tasks to queue
        await client.post('http://localhost:8000/api/tasks', json={'pickup_station':'P1', 'delivery_station':'D1'})
        await client.post('http://localhost:8000/api/tasks', json={'pickup_station':'P2', 'delivery_station':'D2'})
        
        # 3. Start sim
        res = await client.post('http://localhost:8000/api/simulation/start')
        print('Start:', res.json())

    # 4. Listen to WS
    async with websockets.connect('ws://localhost:8000/ws/fleet') as ws:
        for _ in range(15):
            msg = await ws.recv()
            payload = json.loads(msg)
            if payload['type'] == 'STATE_UPDATE':
                data = payload['data']
                ntick = data['nexfleet']['tick']
                r_n = data['nexfleet']['robots'][0]
                r_t = data['traditional']['robots'][0]
                print(f"Tick: {ntick} | TRAD AMR-01: {r_t['status']} BAT:{r_t['battery']:.1f} | NEX AMR-01: {r_n['status']} BAT:{r_n['battery']:.1f}")

if __name__ == '__main__':
    asyncio.run(test_flow())

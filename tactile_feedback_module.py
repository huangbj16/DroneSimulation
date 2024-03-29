import asyncio
from bleak import BleakScanner, BleakClient
import json
import numpy as np
import threading
import time

MOTOR_UUID = 'f22535de-5375-44bd-8ca9-d0ea9ff9e410'
filename = "ActuatorDirections_norm.txt"

def find_three_largest_indices(lst):
    # Use enumeration to keep track of indices and sort based on values
    sorted_indices = sorted(enumerate(lst), key=lambda x: x[1], reverse=True)
    # Extract the indices of the three largest numbers
    indices_of_largest = [index for index, value in sorted_indices[:2]]
    return indices_of_largest

class TactileFeedbackModule:

    def __init__(self) -> None:
        self.category_num = 32
        self.buck_ids = [0, 30, 120, 150]
        self.motor_ids = []
        self.last_vibs = [-1 for _ in range(self.category_num)]
        self.curret_vibs = [-1 for _ in range(self.category_num)]
        self.motor_directions = []
        self.read_motor_file()
        asyncio.run(self.bluetooth_init())
        # Initialize Bluetooth in a separate thread
        # self.bluetooth_thread = threading.Thread(target=self.run_bluetooth_in_thread, daemon=True)
        # self.bluetooth_thread.start()

    def read_motor_file(self):
        ### because the 3D positions are marked in Unity, need to convert to Unreal.
        with open(filename, "r") as f:
            lines = f.readlines()
            for line in lines:
                data = json.loads(line)
                self.motor_ids.append(data["motor_id"])
                direction = data["direction"]
                self.motor_directions.append([direction[2], direction[0], direction[1]])
        self.motor_directions = np.round(np.array(self.motor_directions), 3)
        print("motor ids = ", self.motor_ids)
        print("motor directinos = ", self.motor_directions)

    # def run_bluetooth_in_thread(self):
    #     # Create a new event loop for the thread
    #     loop = asyncio.new_event_loop()
    #     asyncio.set_event_loop(loop)

    #     # Run the asynchronous Bluetooth init function in the new loop
    #     loop.run_until_complete(self.bluetooth_init())

    async def bluetooth_init(self):
        devices = await BleakScanner.discover()
        for d in devices:
            # print('device name = ', d.name)
            if d.name != None:
                if d.name == 'FEATHER_ESP32':
                    print('feather device found!!!')
                    self.client = BleakClient(d.address)
                    try:
                        await self.client.connect()
                        print(f'BLE connected to {d.address}')
                        val = await self.client.read_gatt_char(MOTOR_UUID)
                        print('Motor read = ', val)
                        for buck_addr in self.buck_ids:
                            command = {
                                'addr':buck_addr, 
                                'mode':1,
                                'duty':1, # default
                                'freq':3, # default
                                'wave':1, # default
                            }
                            # turn on the buck converters
                            output = bytearray(json.dumps(command), 'utf-8')
                            print(output)
                            await self.client.write_gatt_char(MOTOR_UUID,  output)
                    except:
                        await self.client.disconnect()

    # command in json format
    def set_vibration(self, addr, duty):
        # mark the motor as vibrating
        self.curret_vibs[self.motor_ids.index(addr)] = duty

    async def set_motor(self, command):
        output = bytearray(json.dumps(command), 'utf-8')
        # print(output)
        # start = time.time()
        await self.client.write_gatt_char(MOTOR_UUID,  output)
        # print(time.time()-start)

    def flush_update(self):
        count = 0
        max_indices = find_three_largest_indices(self.curret_vibs)
        # print(self.curret_vibs)
        for i in range(self.category_num):
            if not i in max_indices:
                self.curret_vibs[i] = -1
            if self.last_vibs[i] != self.curret_vibs[i]: # need update
                count += 1
                if self.curret_vibs[i] == -1:
                    command = {
                        'addr':self.motor_ids[i], 
                        'mode':0,
                        'duty':0, # default
                        'freq':2, # default
                        'wave':0, # default
                    }
                    asyncio.run(self.set_motor(command))
                    asyncio.run(self.set_motor(command))
                else:
                    command = {
                        'addr':self.motor_ids[i], 
                        'mode':1,
                        'duty':self.curret_vibs[i], # default
                        'freq':2, # default
                        'wave':0, # default
                    }
                    asyncio.run(self.set_motor(command))
        # update the vibs
        self.last_vibs = self.curret_vibs
        self.curret_vibs = [-1 for _ in range(self.category_num)]    
        # print("flush count = ", count)


if __name__ == "__main__":
    tactile_module = TactileFeedbackModule()
    tactile_module.set_vibration(1, 15)
    tactile_module.set_vibration(31, 15)
    tactile_module.flush_update()
    time.sleep(2)
    tactile_module.flush_update()

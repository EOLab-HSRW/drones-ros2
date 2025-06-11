"""
This is a very simple example on How to Takeoff and Land.

Known limitations:
- Limitations in multi-drone scenarios: See https://github.com/mavlink/MAVSDK-Python/issues/102
- No discovery

This example is taken from: https://github.com/mavlink/MAVSDK-Python/blob/main/examples/takeoff_and_land.py

See this for a example including telemetry: https://github.com/mavlink/MAVSDK-Python/blob/main/examples/telemetry_takeoff_and_land.py 
"""

import asyncio
from mavsdk import System

async def main():

    drone = System()
    # For example to connect to a drone in simulation
    await drone.connect(system_address="udp://:14540")

    status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("-- Landing")
    await drone.action.land()

    status_text_task.cancel()


async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    asyncio.run(main())

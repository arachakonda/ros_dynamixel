# Import the required libraries
from dynamixel_sdk import *                    # Import the Dynamixel SDK library

# Define the relevant constants
PORT_NAME = '/dev/ttyUSB0'                     # Set the port name
BAUDRATE = 57600                               # Set the baudrate
PROTOCOL_VERSION = 2.0                         # Set the protocol version
DXL_IDS = [1]                         # Set the IDs of the motors to read from
# Initialize the port and packet handler
portHandler = PortHandler(PORT_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open the port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")

# Set the baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")

# Initialize the GroupSyncRead
groupSyncRead = GroupSyncRead(portHandler, packetHandler, 128, 8)

# Add the Dynamixel IDs to the GroupSyncRead
for dxl_id in DXL_IDS:
    dxl_addparam_result = groupSyncRead.addParam(dxl_id)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % dxl_id)
        quit()

# Sync read the current positions and velocities of the motors
dxl_comm_result = groupSyncRead.txRxPacket()
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

# Loop through the motor IDs to print their current position and velocity
for dxl_id in DXL_IDS:
    dxl_getdata_result = groupSyncRead.isAvailable(dxl_id, 128, 8)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupSyncRead getdata failed" % dxl_id)
        quit()
    dxl_current_velocity = groupSyncRead.getData(dxl_id, 128, 4)
    dxl_current_position = groupSyncRead.getData(dxl_id, 132, 4)
    print("[ID:%03d] Current Position: %d Current Velocity: %d" % (dxl_id, dxl_current_position, dxl_current_velocity))

# Close the port
portHandler.closePort()

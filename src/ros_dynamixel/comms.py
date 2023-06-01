#!/usr/bin/env python3
from dynamixel_sdk import *
from ros_dynamixel.msg import *
from ros_dynamixel.vars import *

def get_present_current(id):
    dxl_present_pos, dxl_comm_resut, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_CURRENT)
    return dxl_present_pos

def get_present_pos(id):
    dxl_present_pos, dxl_comm_resut, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POS)
    return dxl_present_pos

def enable_torque(portHandler, packetHandler, id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % id)

def add_paramBR(groupBulkRead, id, param_addr, param_len):
    # Add parameter storage for Dynamixel#1 present position
    dxl_addparam_result = groupBulkRead.addParam(id, param_addr, param_len)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkRead addparam failed" % id)
        quit()

def bulkRead(groupBulkRead, packetHandler):
    dxl_comm_result = groupBulkRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

def extractData(groupBulkRead, id, param_addr, param_len):
    # Check if groupbulkread data of Dynamixel#1 is available
    dxl_getdata_result = groupBulkRead.isAvailable(id, param_addr, param_len)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkRead getdata failed" % id)
        quit()
    dxl_data = groupBulkRead.getData(id, param_addr, param_len)

    return dxl_data

def clearBR(groupBulkRead):
    groupBulkRead.clearParam()

def clearBW(groupBulkWrite):
    groupBulkWrite.clearParam()

def add_paramBW(groupBulkWrite, id, param_addr, param_len, param_val):
    # Allocate goal position value into byte array
    if param_len == 4:
        param_val_array = [DXL_LOBYTE(DXL_LOWORD(param_val)), 
                            DXL_HIBYTE(DXL_LOWORD(param_val)), 
                            DXL_LOBYTE(DXL_HIWORD(param_val)), 
                            DXL_HIBYTE(DXL_HIWORD(param_val))]
    elif param_len == 2:
        param_val_array = [DXL_LOBYTE(param_val), 
                            DXL_HIBYTE(param_val)]
    
    elif param_len == 1:
        param_val_array=[param_val]

    # Add Dynamixel#1 parameter value to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(id, param_addr, param_len, param_val_array)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkWrite addparam failed" % id)
        quit()

def bulkWrite(groupBulkWrite, packetHandler):
    # Bulkwrite goal position and LED value
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    
def disable_torque(portHandler, packetHandler, id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully disconnected" % id)

def setOpMode(portHandler, packetHandler, id, mode):
    disable_torque(portHandler, packetHandler, id)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OP_MODE, mode)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully shifted to mode %d" % (id,mode))

    enable_torque(portHandler, packetHandler, id)

def setProfVel(portHandler, packetHandler, id, val):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_PROF_VEL, val)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pass
        #print("Dynamixel#%d has been successfully fixed to velocity profile maximum %d" % (id,val))
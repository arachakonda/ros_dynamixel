#!/usr/bin/env python3
from dynamixel_sdk import *
from ros_dynamixel.msg import *
from ros_dynamixel.vars import *

def get_present_current(id, portHandler, packetHandler):
    dxl_present_curr, dxl_comm_resut, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_CURR)
    return dxl_present_curr

def get_present_velocity(id, portHandler, packetHandler):
    dxl_present_vel, dxl_comm_resut, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_VEL)
    return dxl_present_vel

def get_present_pos(id, portHandler, packetHandler):
    dxl_present_pos, dxl_comm_resut, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POS)
    return dxl_present_pos

def enable_torque(portHandler, packetHandler, id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d Torque is enabled" % id)

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

def syncRead(groupSyncRead, packetHandler):
    # Sync read the current positions and velocities of the motors
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        quit()

def checkDataAvailable(groupSyncRead, DXL_IDS, startAddr, dataLen):
    for id in DXL_IDS:
        dxl_getdata_result = groupSyncRead.isAvailable(id, startAddr, dataLen)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % (id))
            quit()


def extractData(groupBulkRead, id, param_addr, param_len):
    # Check if groupbulkread data of Dynamixel#1 is available
    dxl_getdata_result = groupBulkRead.isAvailable(id, param_addr, param_len)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkRead getdata failed" % id)
        quit()
    dxl_data = groupBulkRead.getData(id, param_addr, param_len)

    if param_addr == ADDR_PRESENT_VEL:
        if dxl_data > 2047:
            #take 2's complement of 32 bit number
            dxl_data = dxl_data - 4294967296
    if param_addr == ADDR_PRESENT_CURR:
        if dxl_data > 2047:
            #take 2's complement of 32 bit number
            dxl_data = dxl_data - 4294967296
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
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,  id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d Torque is disabled" % id)

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

def add_SyncReadIDs(groupSyncRead, DXL_IDS):
    for id in DXL_IDS:
        try:
            groupSyncRead.addParam(id)
            print("SyncRead ID %d added" % id)
        except:
            print("Motor [ID:%03d] groupSyncRead addition failed" % id)

def add_SyncWriteIDParams(groupSyncWrite, DXL_IDS, param_addr, param_len):
    for id in DXL_IDS:
        try:
            groupSyncWrite.addParam(id, param_len, param_addr)
            print("SyncWrite ID %d added" % id)
        except:
            print("Motor [ID:%03d] groupSyncWrite addition failed" % id)

def clearSR(groupSyncRead):
    groupSyncRead.clearParam()

def init_ids(pos, vel, curr, DXL_IDS):
    for i in range(0,len(DXL_IDS)):
        if(i == 0):
            pos.id_mcpf = DXL_IDS[i]
            vel.id_mcpf = DXL_IDS[i]
            curr.id_mcpf = DXL_IDS[i]
        elif(i == 1):
            pos.id_mcpa = DXL_IDS[i]
            vel.id_mcpa = DXL_IDS[i]
            curr.id_mcpa = DXL_IDS[i]
        elif(i == 2):
            pos.id_pip = DXL_IDS[i]
            vel.id_pip = DXL_IDS[i]
            curr.id_pip = DXL_IDS[i]
        elif(i == 3):
            pos.id_dip = DXL_IDS[i]
            vel.id_dip = DXL_IDS[i]
            curr.id_dip = DXL_IDS[i]

def extractSyncReadData(groupSyncRead, DXL_IDS, pos, vel, curr):
    for i in range(0, len(DXL_IDS)):
        try:
            if(i == 0):
                pos.pos_mcpf = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_POS, LEN_PRESENT_POS)
                vel.vel_mcpf = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_VEL, LEN_PRESENT_VEL)
                curr.curr_mcpf = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_CURR, LEN_PRESENT_CURR)
            elif(i == 1):
                pos.pos_mcpa = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_POS, LEN_PRESENT_POS)
                vel.vel_mcpa = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_VEL, LEN_PRESENT_VEL)
                curr.curr_mcpa = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_CURR, LEN_PRESENT_CURR)
            elif(i == 2):
                pos.pos_pip = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_POS, LEN_PRESENT_POS)
                vel.vel_pip = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_VEL, LEN_PRESENT_VEL)
                curr.curr_pip = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_CURR, LEN_PRESENT_CURR)
            elif(i == 3):
                pos.pos_dip = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_POS, LEN_PRESENT_POS)
                vel.vel_dip = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_VEL, LEN_PRESENT_VEL)
                curr.curr_dip = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_CURR, LEN_PRESENT_CURR)
        except:
            print("[ID:%03d] groupSyncRead getdata failed" % (id))
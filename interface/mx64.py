from dynamixel_sdk import *
import numpy as np
#N - Sync
#B - Bulk
class Mx64:
    def __init__(self, _port):
        self._port = _port
        self.portHandler = PortHandler(self._port)
        self.packetHandler = PacketHandler(2.0)
        self.groupSyncRead = None
        self.groupSyncWrite = None
        self.groupBulkRead = None
        self.groupBulkWrite = None
        self.start()
    
    def start(self):
        print("Opening port {}".format(self._port))
        self.portHandler.openPort()
        self.portHandler.setBaudRate(57600)
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, 132, 4)#read sync present position
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, 116, 4)#write sync goal position
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)
        self.groupBulkWrite = GroupBulkWrite(self.portHandler, self.packetHandler)

    def end(self):
        self.portHandler.closePort()

    def setGoalPosition(self, id, pos):
        self.packetHandler.write4ByteTxOnly(self.portHandler, id, 116, pos)
    
    def setGoalVelocity(self, id, vel):
        self.packetHandler.write4ByteTxOnly(self.portHandler, id, 104, vel)

    def setTorqueEnable(self, id, tor):
        self.packetHandler.write1ByteTxOnly(self.portHandler, id, 64, tor)

    def setNGoalPosition(self, nid, ndata):
        self.groupSyncWrite.clearParam()
        self.groupSyncWrite.start_address = 116
        self.groupSyncWrite.data_length = 4

        for id, val in self.zipID_Data(nid, ndata):
            param = [   DXL_LOBYTE(DXL_LOWORD(val)),
                        DXL_HIBYTE(DXL_LOWORD(val)),
                        DXL_LOBYTE(DXL_HIWORD(val)),
                        DXL_HIBYTE(DXL_HIWORD(val)) ]
            self.groupSyncWrite.addParam(id, param)
        self.groupSyncWrite.txPacket()
    
    def setNGoalVelocity(self, nid, ndata):
        self.groupSyncWrite.clearParam()
        self.groupSyncWrite.start_address = 104
        self.groupSyncWrite.data_length = 4
        for id, val in self.zipID_Data(nid, ndata):
            param = [   DXL_LOBYTE(DXL_LOWORD(val)),
                        DXL_HIBYTE(DXL_LOWORD(val)),
                        DXL_LOBYTE(DXL_HIWORD(val)),
                        DXL_HIBYTE(DXL_HIWORD(val)) ]
        self.groupSyncWrite.addParam(id, param)
        self.groupSyncWrite.txPacket()

    def zipID_Data(self, nid, ndata):
        if len(nid) != len(ndata):
            print("Ids and Data not match")
            return
        for i in range(0, len(nid)):
            yield nid[i], ndata[i]


    def setNTorqueEnable(self, nid, ndata):
        self.groupSyncWrite.clearParam()
        self.groupSyncWrite.start_address = 64
        self.groupSyncWrite.data_length = 1
        for id, val in self.zipID_Data(nid, ndata):
            self.groupSyncWrite.addParam(id, [val])
        self.groupSyncWrite.txPacket()

    def getPresentPosition(self, id):
        return self.packetHandler.read4ByteTxRx(self.portHandler, id, 132)[0]

    def getTorqueEnable(self, id):
        return self.packetHandler.read1ByteTxRx(self.portHandler, id, 64)[0]

    def getNPresentPosition(self, nid):
        self.groupSyncRead.clearParam()
        self.groupSyncRead.start_address = 132
        self.groupSyncRead.data_length = 4
        for idx in nid:
            self.groupSyncRead.addParam(idx)
        #transmit to tx and get rx data
        self.groupSyncRead.txRxPacket()
        return [ [self.groupSyncRead.getData(id, 132, 4) for id in idx] for idx in nid ]

    def angleToPosition(self, angle):
        return int(4096/360*angle)

    def positionToAngle(self, pos):
        return 360/4096*pos

    def setGoalAngle(self, id, angle):
        pos = self.angleToPosition(angle)
        self.setGoalPosition(id, pos)
    
    def setNGoalAngle(self, nid, nangle):
        npos = [[self.angleToPosition(angle) for angle in anglex] for anglex in nangle]
        self.setNGoalPosition(nid, npos)
    
    def setLimitPosition(self, id, minPos, maxPos): #id - min position - max position
        self.packetHandler.write4ByteTxOnly(self.portHandler, id, 52, minPos)
        self.packetHandler.write4ByteTxOnly(self.portHandler, id, 48, maxPos)
    
    def setDriveMode(self, id, mode):
        self.packetHandler.write1ByteTxOnly(self.portHandler, id, 10, mode)

# mx = Mx64('/dev/ttyUSB0')
# mx.setTorqueEnable(1, 1)
# for i in range(1,12):
#     print(mx.getTorqueEnable(i))
# # mx.setTorqueEnable(1, 1)
# # mx.setTorqueEnable(2, 1)
# # mx.setTorqueEnable(3, 1)
# # mx.setNTorqueEnable([1,2,3,4,5,6,7,8], [1,1,1,1,1,1,1,1])
# # for i in range(10):
# #     mx.setNGoalPosition([1,2,3,4,5,6,7,8], [2046,2046,2046,2046,2046,2046,2046,2046])
# #     time.sleep(1)
# #     mx.setNGoalPosition([1,2,3,4,5,6,7,8], [2300,2300,2300,2300,2300,2300,2300,2300])
# #     time.sleep(1)

# while 1:
#     print(mx.getPresentPosition(1))
#     time.sleep(0.1)

# mx.end()


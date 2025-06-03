def modbusCrc(msg:str) -> int:
    crc = 0xFFFF
    for n in range(len(msg)):
        crc ^= msg[n]
        for i in range(8):
            if crc & 1:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    
    crcl = (crc >> 8) & 0xFF
    crch = crc & 0xFF
    return crcl, crch, crc
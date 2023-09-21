import math

MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
#AK8963 registers
AK8963_ADDR  = 0x0C
AK8963_ST1   = 0x02
HXH          = 0x04
HYH          = 0x06
HZH          = 0x08
AK8963_ST2   = 0x09
AK8963_CNTL  = 0x0A

class Gyroscope():
    def __init__(self, bus , sense_index = 0) -> None:
        self.bus = bus
        self.sense_index = sense_index
        self.config_config_val = [250.0,500.0,1000.0,2000.0]

    def read(self):
        gyro_x = self.reader(GYRO_XOUT_H)
        gyro_y = self.reader(GYRO_YOUT_H)
        gyro_z = self.reader(GYRO_ZOUT_H)
        return gyro_x, gyro_y, gyro_z
    
    def angles(self):
        gyro_x, gyro_y, gyro_z = self.read()
        wX = (gyro_x/(2.0**15.0))*self.config_config_val[self.sense_index]
        wY = (gyro_y/(2.0**15.0))*self.config_config_val[self.sense_index]
        wZ = (gyro_z/(2.0**15.0))*self.config_config_val[self.sense_index]

        return wX, wY,wZ

    def reader(self, register):
        # read accel and gyro values
        high = self.bus.read_byte_data(MPU6050_ADDR, register)
        low = self.bus.read_byte_data(MPU6050_ADDR, register+1)
        # combine higha and low for unsigned bit value
        value = ((high << 8) | low)
        # convert to +- value
        if(value > 32768):
            value -= 65536
        return value

class Accelerometer():
    def __init__(self,bus, sense_index = 0) -> None:
        self.bus = bus
        self.sense_index = sense_index
        self.sense_config_val = [2.0,4.0,8.0,16.0]
        pass

    def read(self):
        # raw acceleration bits
        acc_x = self.reader(ACCEL_XOUT_H)
        acc_y = self.reader(ACCEL_YOUT_H)
        acc_z = self.reader(ACCEL_ZOUT_H)
        return acc_x, acc_y, acc_z

    def normalize(self):
        acc_x, acc_y, acc_z = self.read()
        a_x = (acc_x/(2.0**15.0))*self.sense_config_val[self.sense_index]
        a_y = (acc_y/(2.0**15.0))*self.sense_config_val[self.sense_index]
        a_z = (acc_z/(2.0**15.0))*self.sense_config_val[self.sense_index]
        return a_x, a_y, a_z

    def angles(self):
        a_x, a_y, a_z = self.normalize()
        x = math.atan2 ( a_y, math.sqrt ( a_z *  a_z  + a_x * a_x)) * (180 / math.pi)
        y = math.atan2 (- a_x, math.sqrt( a_z *  a_z + a_y * a_y)) * (180 / math.pi)
        z = math.atan2 (math.sqrt( a_x *  a_x + a_y * a_y), a_z) * (180 / math.pi)
        return x, y, z

    def reader(self, register):
        # read accel and gyro values
        high = self.bus.read_byte_data(MPU6050_ADDR, register)
        low = self.bus.read_byte_data(MPU6050_ADDR, register+1)
        # combine higha and low for unsigned bit value
        value = ((high << 8) | low)		
        # convert to +- value
        if(value > 32768):
            value -= 65536
        return value

class Magnetometer():
    def __init__(self, bus,sense_index =0) -> None:
        self.bus = bus
        self.sense_index = sense_index
        self.sense_config_val = [4900.0]
        pass

    def read(self):
        loop_count = 0
        while 1:
            mag_x = self.reader(HXH)
            mag_y = self.reader(HYH)
            mag_z = self.reader(HZH)
            if bin(bus.read_byte_data(AK8963_ADDR,AK8963_ST2))=='0b10000':
                return mag_x, mag_y, mag_z
            loop_count+=1

    def normalize(self):
        mag_x, mag_y, mag_z = self.read()
        m_x = (mag_x/(2.0**15.0))*self.sense_config_val[self.sense_index]
        m_y = (mag_y/(2.0**15.0))*self.sense_config_val[self.sense_index]
        m_z = (mag_z/(2.0**15.0))*self.sense_config_val[self.sense_index]
        return m_x, m_y, m_z
	
    def heading(self):
        m_x, m_y, m_z= self.normalize()
        heading = math.atan2( m_x, m_y ) * (180/ math.pi) 
        return m_x, m_y, m_z, heading

    def reader(self, register):
        # read magnetometer values
        low = self.bus.read_byte_data(AK8963_ADDR, register-1)
        high = self.bus.read_byte_data(AK8963_ADDR, register)
        # combine higha and low for unsigned bit value
        value = ((high << 8) | low)
        # convert to +- value
        if(value > 32768):
            value -= 65536
        return value

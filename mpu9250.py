'''
This script must use with registermap.py
'''
import time, numpy as np, registermap as rgm
import smbus # type: ignore

Wire = smbus.SMBus(1)
 
def setup(frequency:int = 100, verbose:bool = False):
    '''
    Setup sensor for I2C read/write, check sensor connection and calibrate the sensor.
    '''
    if verbose:
      print("Initialize...\n")
      print("MPU6500: Reset...\n")

    
    # Check connection of MPU6500.
    is_connect(model = "MPU6500", verbose = verbose)
    # Reset all registers of the MPU6500.
    writebyte(rgm.MPU6500_ADDRESS, rgm.PWR_MGMT_1, np.array(0x40, dtype = np.uint8))
    time.sleep(0.1)
    writebyte(rgm.MPU6500_ADDRESS, rgm.PWR_MGMT_1, np.array(0x80, dtype = np.uint8))
    time.sleep(0.1)
    writebyte(rgm.MPU6500_ADDRESS, rgm.PWR_MGMT_1, np.array(0x00, dtype = np.uint8))
    time.sleep(0.1)

    if verbose:
      print("MPU9250: Enable AK8963\n")
    # Enable AK8963 in MPU9250.
    writebyte(rgm.MPU6500_ADDRESS, rgm.INT_PIN_CFG, np.array(0x02, dtype = np.uint8))
    time.sleep(0.1)
    writebyte(rgm.MPU6500_ADDRESS, rgm.I2C_SLV0_ADDR, np.array(rgm.AK8963_ADDRESS, dtype = np.uint8))
    time.sleep(0.1)
    # Check connection of AK8963.
    is_connect(model = "AK8963", verbose = verbose)
      
    if verbose:
      print("AK8963: Reset...\n")
    # Reset all registers of the AK8963.
    writebyte(rgm.AK8963_ADDRESS, rgm.CNTL1, np.array(0x00, dtype = np.uint8))
    time.sleep(0.1)
    writebyte(rgm.AK8963_ADDRESS, rgm.CNTL2, np.array(0x01, dtype = np.uint8))
    time.sleep(0.1)

    if verbose:
      print("MPU6500: Set accelerometer range +/-2 g\n")
      print("MPU6500: Set Gyroscope range +/-250 Degrees Per Second\n")
      print("AK8963: Magnetometer range already +/-4800 microTesla\n")
    # Configure the range of the sensor.
    # For Magnetometer (AK8963) it has only one range.
    writebyte(rgm.MPU6500_ADDRESS, rgm.ACCEL_CONFIG, np.array(rgm.ACCEL_FULL_SCALE_2G, dtype = np.uint8))
    time.sleep(0.1)
    writebyte(rgm.MPU6500_ADDRESS, rgm.GYRO_CONFIG, np.array(rgm.GYRO_FULL_SCALE_250DPS, dtype = np.uint8))
    time.sleep(0.1)

    if frequency == 8:
      if verbose:
        print("MPU6500: Set sample rate to 8Hz\n")
        print("AK8963: Set sample rate to 8Hz\n")
      # Configure the sample rate.
      # Range of MPU6500 sample rate is 8Hz to 8KHz.
      # Range of AK8963 sample rate is 8Hz to 100Hz.
      writebyte(rgm.MPU6500_ADDRESS, rgm.SMPLRT_DIV, np.array(CAL_SMPLRT(8), dtype = np.uint8))
      time.sleep(0.1)
      writebyte(rgm.AK8963_ADDRESS, rgm.CNTL1, np.array(rgm.AK8963_CNTL1_CMM1, dtype = np.uint8))
      time.sleep(0.1)
    elif frequency == 100:
      if verbose:
        print("MPU6500: Set sample rate to 100Hz\n")
        print("AK8963: Set sample rate to 100Hz\n")
      # Configure the sample rate.
      # Range of MPU6500 sample rate is 8Hz to 8KHz.
      # Range of AK8963 sample rate is 8Hz to 100Hz.
      writebyte(rgm.MPU6500_ADDRESS, rgm.SMPLRT_DIV, CAL_SMPLRT(100))
      time.sleep(0.1)
      writebyte(rgm.AK8963_ADDRESS, rgm.CNTL1, np.array(rgm.AK8963_CNTL1_CMM2, dtype = np.uint8))
      time.sleep(0.1)

    if verbose:
      print("MPU6500: Set digital low-pass filter for accelerometer and gyroscope\n")
    # Configure the digital low-pass filter.
    writebyte(rgm.MPU6500_ADDRESS, rgm.CONFIG, np.array(rgm.MPU6500_DLPF_CFG_6, dtype = np.uint8))
    time.sleep(0.1)
    writebyte(rgm.MPU6500_ADDRESS, rgm.ACCEL_CONFIG_2, np.array(rgm.MPU6500_DLPF_CFG_6, dtype = np.uint8))
    time.sleep(0.1)
  
  # This function read WHO_I_AM of MPU6500 and WIA of AK8963 to verify sensor model and check connection.

def is_connect(model, verbose:bool = False):
    '''
    Check connection of sensor.
    '''
    if model == "MPU6500":
      if verbose:
        print("MPU6500: I am", hex(readbyte(rgm.MPU6500_ADDRESS, rgm.WHO_AM_I)), "\n")
        print("MPU6500: I should be 0x71\n")

      if(readbyte(rgm.MPU6500_ADDRESS, rgm.WHO_AM_I) == 0x71):
        if verbose:
          print("MPU6500: Online\n")
      else:
        print("MPU6500: Offline\n")
        print("MPU9250: Something wrong...!\n")
        while(True):
          pass

    else:
      if verbose:
        print("AK8963: I am", hex(readbyte(rgm.AK8963_ADDRESS, rgm.WIA)), "\n")
        print("AK8963: I should be 0x48\n")

      if(readbyte(rgm.AK8963_ADDRESS, rgm.WIA) == 0x48):
        if verbose:
          print("AK8963: Online\n")
      else:
        print("AK8963: Offline\n")
        print("AK8963: Something wrong...!\n")
        while(True):
          pass
  
def get_MARG(filter_mode:str = "MARG", IMU_bias = np.array([0, 0, 0, 0, 0, 0], dtype = np.float16), MAG_bias = np.array([[0, 0, 0], [1, 1, 1]], dtype = np.float16)):
  '''
  Read accelerometer, gyroscope and magnetometer data from the sensor.
  '''
  MARG = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0], dtype = np.float16)
  MPU6500_RAW = readbytes(rgm.MPU6500_ADDRESS, rgm.ACCEL_XOUT_H, 14)
  MARG[0] = sign_data(MPU6500_RAW[0] << 8 | MPU6500_RAW[1]) / rgm.ACCEL_SCALING_FACTOR
  MARG[1] = -sign_data(MPU6500_RAW[4] << 8 | MPU6500_RAW[5]) / rgm.ACCEL_SCALING_FACTOR
  MARG[2] = sign_data(MPU6500_RAW[2] << 8 | MPU6500_RAW[3]) / rgm.ACCEL_SCALING_FACTOR
  MARG[3] = sign_data(MPU6500_RAW[8] << 8 | MPU6500_RAW[9]) / rgm.GYRO_SCALING_FACTOR
  MARG[4] = -sign_data(MPU6500_RAW[12] << 8 | MPU6500_RAW[13]) / rgm.GYRO_SCALING_FACTOR
  MARG[5] = sign_data(MPU6500_RAW[10] << 8 | MPU6500_RAW[11]) / rgm.GYRO_SCALING_FACTOR
  if filter_mode == "MARG":
    while((readbyte(rgm.AK8963_ADDRESS, rgm.ST1) & 0x1) != 1):
      pass
    AK8963_RAW = readbytes(rgm.AK8963_ADDRESS, rgm.HXL, 7)
    MARG[6] = ((sign_data(AK8963_RAW[2] | AK8963_RAW[3] << 8) / rgm.MAG_SCALING_FACTOR) - MAG_bias[0][0]) * MAG_bias[1][0]
    MARG[7] = ((sign_data(AK8963_RAW[4] | AK8963_RAW[5] << 8) / rgm.MAG_SCALING_FACTOR) - MAG_bias[0][1]) * MAG_bias[1][1]
    MARG[8] = ((sign_data(AK8963_RAW[0] | AK8963_RAW[1] << 8) / rgm.MAG_SCALING_FACTOR) - MAG_bias[0][2]) * MAG_bias[1][2]
  for i in range(6):
    MARG[i] = MARG[i] - IMU_bias[i]
    
  for i in range(3):
    if(MARG[i] > 1):
      MARG[i] = -MARG[i] + 2
    elif(MARG[i] < -1):
      MARG[i] = -MARG[i] - 2
  return MARG

def readbyte(addr:int, reg:int):
  '''
  Function to read register over I2C.
  This function read 1 byte from I2C device.
  addr : Device's I2C address.
  reg : Register address.
  '''
  return np.array(Wire.read_byte_data(addr, reg), dtype = np.uint8)

def readbytes(addr:int, reg:int, nbytes:int):
  '''
  Function to read register over I2C.
  This function read n bytes from I2C device.
  addr : Device's I2C address.
  reg : Register address.
  nbytes : Read data n bytes start from register address.
  '''
  return np.array(Wire.read_i2c_block_data(addr, reg, nbytes), dtype = np.uint8)

def writebyte(addr:int, reg:int, data):
  '''
  Function to write a value to a register over I2C
  addr : Device's I2C address.
  reg : Register address.
  data : Data that you want to write to register address.
  '''
  Wire.write_byte_data(addr, reg, np.array(data, dtype = np.uint8))

def CAL_SMPLRT(samplerate:int = 8):
  '''
  Calculate sample rate for MPU6500.
  '''
  # internal clock frequency of the MPU6500
  internalClock = 1000
  # return sample rate in byte format
  return np.array(((internalClock / samplerate) - 1), dtype = np.uint8)

def sign_data(data:np.uint16):
  '''
  Function to sign data of 2 byte.
  data : 16-bit data
  '''
  # Data is negative
  if data >= 0x8000:
    return -((0xFFFF - data) + 1)
  else:
    # Data is positive
    return data
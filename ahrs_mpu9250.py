'''
This script requires the following libraries:
- ahrs
- numpy
you can install by using command "pip install ahrs numpy" in the terminal.
'''
import time, numpy as np, ahrs, mpu9250 as imu
import smbus # type: ignore

DPS2RPS = 0.0174532925  # deg/sec to rad/sec ratio

class MPU9250:
  def __init__(self, bus:int = -1, frequency:int = 100, filter_mode:str = "MARG", gain:float = 0.041, mag_calibrationtime:float = 30, ahrs_stabilizetime:float = 30, calibrate_nsample:int = 100, IMU_bias = np.array([0, 0, 0, 0, 0, 0], dtype = np.float16), MAG_bias = np.array([[0, 0, 0], [1, 1, 1]], dtype = np.float16), disable_calibrate:bool = False, verbose:bool = False):
    '''
    bus : Bus number that sensor connected to ``TCA9548A``.
    frequency : Select sensor frequency between ``8Hz`` and ``100Hz``.
    filter_mode : Select between ``IMU`` and ``MARG``. 

    ``IMU``  --> ``acc`` and ``gyr``
    ``MARG`` --> ``acc``, ``gyr`` and ``mag``

    gain : Default gain for ``MadgwickIMU`` is ``0.033``, ``MadgwickMARG`` is ``0.041``.
    ahrs_stabilizetime : Stabilize ``MadgwickMARG`` algorithm for ``n`` sec.
    calibrate_nsample : Collect ``n`` sample and offset the output to be close to zero.
    IMU_bias : np.array([ax, ay, az, gx, gy, gz], dtype = np.float16)
    MAG_bias : np.array([(offset term)[mx, my, mz], (scale term)[mx, my,mz]]
    disable_calibrate : Disable calibration process when setup sensor.
    verbose : Show sensor output to the ``terminal``.
    '''
    self.bus = bus
    if (frequency != 8) and (frequency != 100):
      while (frequency != 8) and (frequency != 100):
        frequency = int(input("Please input frequency '8' Hz or '100' Hz. : "))
      self.frequency = frequency
    else:
      self.frequency = frequency
    if (filter_mode != "IMU") and (filter_mode != "MARG"):
      while (filter_mode != "IMU") and (filter_mode != "MARG"):
        filter_mode = str(input("Please input filter mode 'IMU' or 'MARG' for Madgwick. : "))
      self.filter_mode = filter_mode
    else:
      self.filter_mode = filter_mode
    self.gain = gain
    self.mag_calibrationtime = mag_calibrationtime
    self.ahrs_stabilizetime = ahrs_stabilizetime
    self.calibrate_nsample = calibrate_nsample
    self.disable_calibrate = disable_calibrate
    self.verbose = verbose
    self.MARG = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0], dtype = np.float16)
    self.IMU_bias = IMU_bias
    self.MAG_bias = MAG_bias
    self.AHRS = np.array([0, 0, 0], dtype = np.float16)
    self.AHRS_bias = np.array([0, 0, 0], dtype = np.float16)
    
  def begin(self):
    imu.setup(self.bus, self.frequency, self.verbose)
    if self.disable_calibrate == False:
      if np.array_equal(self.IMU_bias, np.array([0, 0, 0, 0, 0, 0])):
        self.calibrate_IMU()
      if self.filter_mode == "MARG" and np.array_equal(self.MAG_bias, np.array([[0, 0, 0], [1, 1, 1]], dtype = np.float16)):
        self.calibrate_MAG()
    self.get_MARG()
    acc_data = np.array([self.MARG[0], self.MARG[1], self.MARG[2]], dtype = np.float64).reshape(-1, 3)

    if self.filter_mode == "IMU":
      gyr_data = np.array([self.MARG[3], self.MARG[4], self.MARG[5]], dtype = np.float64).reshape(-1, 3) * DPS2RPS
      self.filter = ahrs.filters.Madgwick(gyr = gyr_data, acc = acc_data, frequency = self.frequency, gain = self.gain)
      self.update = self.filter.updateIMU
    elif self.filter_mode == "MARG":
      gyr_data = np.array([self.MARG[3], self.MARG[4], self.MARG[5]], dtype = np.float64).reshape(-1, 3) * DPS2RPS
      mag_data = np.array([self.MARG[6], self.MARG[7], self.MARG[8]], dtype = np.float64).reshape(-1, 3)
      self.filter = ahrs.filters.Madgwick(gyr = gyr_data, acc = acc_data, mag = mag_data, frequency = self.frequency, gain = self.gain)
      self.update = self.filter.updateMARG
    self.q = self.filter.Q[0]
    if self.disable_calibrate == False:
      self.calibrate_AHRS()

  def get_MARG(self):
    self.MARG = imu.get_MARG(bus = self.bus, filter_mode = self.filter_mode, IMU_bias = self.IMU_bias, MAG_bias = self.MAG_bias)

  def calibrate_IMU(self):
    '''
    Try to set zero acc, gyr and mag when start the program.
    '''
    IMU_bias = np.array([0, 0, 0, 0, 0, 0], dtype = np.float16)
    msg = "Calibrating acc and gyr. (0/" + str(self.calibrate_nsample) + ")"
    if self.bus >= 0:
        print("Bus: " + str(self.bus) + " " + msg)
    else:
      print(msg)
    for i in range(self.calibrate_nsample):
      self.get_MARG()
      for j in range(6):
        IMU_bias[j] = IMU_bias[j] + (self.MARG[j] / self.calibrate_nsample)
      msg = "Calibrating acc ang gyr. (" + "%s" %(i + 1) + "/" + "%s" %self.calibrate_nsample + ")"
      if self.bus >= 0:
        print("Bus: " + str(self.bus) + " " + msg)
      else:
        print(msg)
    IMU_bias[2] = IMU_bias[2] - 1
    self.IMU_bias = IMU_bias
    print("IMU Bias: ", self.IMU_bias)

  def calibrate_MAG(self):
    '''
    Try to get mag bias by rotate mag around each axis.
    calibrate_sec : Calibrate time in sec.
    '''
    m = [32767, 32767, 32767, -32767, -32767, -32767] # mx_min,my_min, mz_min, mx_max, my_max, mz_max
    t = time.time()
    print("Calculating mag bias...(0 "+ "/" + "%.2f" %(self.mag_calibrationtime) + ")")
    print("Press 'Ctrl + C' to stop collecting sample process.")
    while time.time() - t < self.mag_calibrationtime:
      self.get_MARG()
      print("Calculating mag bias...(" + "%.2f" %(time.time() - t) + "/" + "%.2f" %(self.mag_calibrationtime) + ")")
      print("Press 'Ctrl + C' to stop collecting sample process.")
      for i in range(3):
        if(self.MARG[i + 6] <= m[i]): #min
          m[i] = self.MARG[i + 6]
        if(self.MARG[i + 6] >= m[i + 3]): #max
          m[i + 3] = self.MARG[i + 6]
    print("\nmx_min:", m[0], " | my_min:", m[1], " | mz_min:", m[2])
    print("mx_max:", m[3], " | my_max:", m[4], " | mz_max:", m[5])
    mx_newoffset = (m[0] + m[3]) / 2
    my_newoffset = (m[1] + m[4]) / 2
    mz_newoffset = (m[2] + m[5]) / 2

    mx_delta_avg = (m[0] - m[3]) / 2
    my_delta_avg = (m[1] - m[4]) / 2
    mz_delta_avg = (m[2] - m[5]) / 2

    m_delta_avg = (mx_delta_avg + my_delta_avg + mz_delta_avg) / 3

    mx_newscale = m_delta_avg / mx_delta_avg
    my_newscale = m_delta_avg / my_delta_avg
    mz_newscale = m_delta_avg / mz_delta_avg
    print("new offset: [mx, my, mz] | [" + str(mx_newoffset) + ", " + str(my_newoffset) + ", " + str(mz_newoffset) + "]")
    print(" new scale: [mx, my, mz] | [" + str(mx_newscale) + ", " + str(my_newscale) + ", " + str(mz_newscale) + "]")
    MAG_bias = np.array([[mx_newoffset, my_newoffset, mz_newoffset], [mx_newscale, my_newscale, mz_newscale]], dtype = np.float16)
    self.MAG_bias = MAG_bias
    print("MAG Bias: ", self.MAG_bias)
    print("wait for 5 sec...")
    time.sleep(5)
  
  def get_AHRS(self):
    '''
    Get roll, pitch and yaw of the sensor.
    '''
    self.get_MARG()
    acc_data = np.array([self.MARG[0], self.MARG[1], self.MARG[2]], dtype = np.float64)
    if self.filter_mode == "IMU":
      gyr_data = np.array([self.MARG[3], self.MARG[4], self.MARG[5]], dtype = np.float64) * DPS2RPS
      self.q = self.update(q = self.q, gyr = gyr_data, acc = acc_data) # type: ignore
    elif self.filter_mode == "AM":
      mag_data = np.array([self.MARG[6], self.MARG[7], self.MARG[8]], dtype = np.float64)
      self.q = self.update(acc = acc_data, mag = mag_data) # type: ignore
    elif self.filter_mode == "MARG":
      gyr_data = np.array([self.MARG[3], self.MARG[4], self.MARG[5]], dtype = np.float64) * DPS2RPS
      mag_data = np.array([self.MARG[6], self.MARG[7], self.MARG[8]], dtype = np.float64)
      self.q = self.update(q = self.q, gyr = gyr_data, acc = acc_data, mag = mag_data) # type: ignore
    self.AHRS = ahrs.Quaternion(self.q).to_angles() * ahrs.RAD2DEG
    for i in range(3):
      self.AHRS[i] = self.AHRS[i] - self.AHRS_bias[i]
    
    if self.verbose:
      msg_ahrs = "Roll: " + "%s" %self.AHRS[0] + " | Pitch: " + "%s" %self.AHRS[1] + " | Yaw: " + "%s" %self.AHRS[2]
      msg_acc = "Ax: " + "%.2f" %self.MARG[0] + " | Ay: " + "%.2f" %self.MARG[1] + " | Az: " + "%.2f" %self.MARG[2]
      msg_gyr = "Gx: " + "%.2f" %self.MARG[3] + " | Gy: " + "%.2f" %self.MARG[4] + " | Gz: "+ "%.2f" %self.MARG[5]
      msg_mag = "Mx: " + "%.2f" %self.MARG[6] + " | My: " + "%.2f" %self.MARG[7] + " | Mz: " + "%.2f" %self.MARG[8]

      if self.bus >= 0:
        if self.filter_mode == "IMU":
          print("Bus: " + "%s" %self.bus + " | " + msg_ahrs + " | " + msg_acc + " | " + msg_gyr)
        elif self.filter_mode == "AM":
          print("Bus: " + "%s" %self.bus + " | " + msg_ahrs + " | " + msg_acc + " | " + msg_mag)
        elif self.filter_mode == "MARG":
          print("Bus: " + "%s" %self.bus + " | " + msg_ahrs + " | " + msg_acc + " | " + msg_gyr + " | " + msg_mag)
      else:
        if self.filter_mode == "IMU":
          print(msg_ahrs + " | " + msg_acc + " | " + msg_gyr)
        elif self.filter_mode == "AM":
          print(msg_ahrs + " | " + msg_acc + " | " + msg_mag)
        elif self.filter_mode == "MARG":
          print(msg_ahrs + " | " + msg_acc + " | " + msg_gyr + " | " + msg_mag)

  def calibrate_AHRS(self):
    '''
    Try to set zero of roll, pitch, yaw when start the program.
    '''
    if self.filter_mode == "MARG":
      t = time.time()
      msg = "Stabilizing AHRS algorithm. (0/" + "%s" %self.ahrs_stabilizetime + ")"
      if self.bus >= 0:
        print("Bus: " + str(self.bus) + " " + msg)
      else:
        print(msg)
      while (time.time() - t) < self.ahrs_stabilizetime:
        self.get_AHRS()
        msg = "Stabilizing AHRS algorithm. (" + "%.2f" %(time.time() - t) + "/" + "%s" %self.ahrs_stabilizetime + ")"
        if self.bus >= 0:
          print("Bus: " + str(self.bus) + " " + msg)
        else:
          print(msg)
    AHRS_bias = np.array([0, 0, 0], dtype = np.float16)
    msg = "Calibrating AHRS. (0/" + "%s" %self.calibrate_nsample + ")"
    if self.bus >= 0:
      print("Bus: " + "%s" %self.bus + " " + msg)
    else:
      print(msg)
    for i in range(self.calibrate_nsample):
      self.get_AHRS()
      for j in range(3):
        AHRS_bias[j] = AHRS_bias[j] + (self.AHRS[j] / self.calibrate_nsample)
      msg = "Calibrating AHRS. (" + "%s" %(i + 1) + "/" + "%s" %self.calibrate_nsample + ")"
      if self.bus >= 0:
        print("Bus: " + str(self.bus) + " " + msg)
      else:
        print(msg)
    self.AHRS_bias = AHRS_bias
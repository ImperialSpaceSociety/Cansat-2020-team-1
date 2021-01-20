
  // Libraries
    #include <Wire.h>
    #define MPU_I2C_ADD 0x68
    #define AK8_I2C_ADD 0x0C
    // MPU9250 Datasshet at: https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
    // MPU9250 Registers at: https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf
    // AK8963  Datasshet at: https://download.mikroe.com/documents/datasheets/ak8963c-datasheet.pdf

  // Global variables
    // MPU9250
      uint8_t I2C_buffer[14];           // general use for MPU9250
      float IMU_Data[10];               // for the 10DoF: Acc XYZ, Gyr XYZ, Mag XYZ, temp C
      float mScaleX, mScaleY, mScaleZ;  // Scaling factors for magnetometer
      float aBias[3] = {0.0, 0.0, 0.0};   // Acc bias
      float gBias[3] = {0.0, 0.0, 0.0};   // Gyr bias
    // AHRS
      // Orientations
        float qMad[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // Madgwick quaternion
        float qMah[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // Mahony   quaternion
        float Mad_Euler[3] = {0.0f,0.0f,0.0f};      // Madgwick euler angles    Yaw, pitch, roll
        float Mah_Euler[3] = {0.0f,0.0f,0.0f};      // Mahony   euler angles
      // Filter Settings
        float GyroMeasError = PI * (40.0f / 180.0f);      // gyroscope measurement error in rads/s (start at 40 deg/s)
        float GyroMeasDrift = PI * (0.0f  / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
        float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta, larger = faster = less accurate
        float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, in Madgwick scheme usually set to a small or zero value
        #define Kp 10.0f                                  // Kp for proportional feedback, Ki for integral
        #define Ki 0.0f
      // Timings      
        float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
        uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
        uint32_t Now = 0;                         // used to calculate integration interval
        float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

    
    // Change these variables
      #define SerialPrint 1    // want serial commands?
      #define I2CScan     0    // want to first scan I2C lines?

      float mBias[3] = {470.0, 120.0, 125.0};               // Mag bias due to environment. Units uT    
      float aSF[3]   = {0.98861, 0.98924, 0.98900};   // Scale factor for acc
      #define MagDeclination 0.0                      // Magnetic declination

  // ------------------------------------------------------------------------------------------------- //
  // --------------------------------------------- Setup --------------------------------------------- //
  // ------------------------------------------------------------------------------------------------- //
  
  void setup() {
    // Setup serial
      if(SerialPrint){Serial.begin(115200);}
      if(SerialPrint){while (!Serial){}}                  // Wait for serial monitor to open
      if(SerialPrint){Serial.println("STM32 MPU-9250");}
      
    // Setup I2C
      Wire.setSDA(PB9);
      Wire.setSCL(PB8);
      Wire.begin();
      
    // Scan I2C lines
      I2C_Scan(0);      // perform I2C scan: 1 = debug printing, 0 = no printing
      
    // Start IMU
      if(SerialPrint){Serial.print("MPU9250... ");}
      int IMU_status = IMU_Begin(0);                      // Start IMU: 1 = debug printing, 0 = no printing
      if(IMU_status){                                     // if any errors, stop program
        if(SerialPrint){Serial.print("ERROR ID: "+String(IMU_status));}
        while(1){};                                                       
      }
      Serial.println("Good");      
      IMU_Zero_Acc(aBias);    // Zero accelerometer
      IMU_Zero_Gyr(gBias);    // Zero gyroscope
    // Other stuff
      delay(100);
  }

  // ------------------------------------------------------------------------------------------------- //
  // --------------------------------------------- Loop  --------------------------------------------- //
  // ------------------------------------------------------------------------------------------------- //

  void loop() {

    // IMU
      IMU_Get_Data_Average(IMU_Data,2);   // Get average of n readings 

    // AHRS
      // Timing things
        Now = micros();
        deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
        lastUpdate = Now;
        sum += deltat; // sum for averaging filter update rate

      // Filter and AHRS
        Mad_Update(IMU_Data[0], IMU_Data[1], IMU_Data[2], IMU_Data[3]*PI/180.0f, IMU_Data[4]*PI/180.0f, IMU_Data[5]*PI/180.0f, IMU_Data[7],  IMU_Data[6], IMU_Data[8]);
        Mah_Update(IMU_Data[0], IMU_Data[1], IMU_Data[2], IMU_Data[3]*PI/180.0f, IMU_Data[4]*PI/180.0f, IMU_Data[5]*PI/180.0f, IMU_Data[7],  IMU_Data[6], IMU_Data[8]);
    
        AHRS_Update(qMad[0],qMad[1],qMad[2],qMad[3],Mad_Euler);
        AHRS_Update(qMah[0],qMah[1],qMah[2],qMah[3],Mah_Euler);

    // Serial print if u wanna ;3
      if(SerialPrint){
        Serial.println("Mad Yaw: "+String(Mad_Euler[0],1)+"\t Pitch: "+String(Mad_Euler[1],1)+"\t Roll: "+String(Mad_Euler[2],1));
        Serial.println("Mah Yaw: "+String(Mah_Euler[0],1)+"\t Pitch: "+String(Mah_Euler[1],1)+"\t Roll: "+String(Mah_Euler[2],1));
        Serial.println("");
      }
      
    /*
    // Serial print if u wanna
      if(SerialPrint){
        Serial.print("Acc XYZ: ");  
        Serial.print(String(IMU_Data[0],2)+"\t"+String(IMU_Data[1],2)+"\t"+String(IMU_Data[2],2)+"\t");
        Serial.print("Gyr XYZ: ");  
        Serial.print(String(IMU_Data[3],2)+"\t"+String(IMU_Data[4],2)+"\t"+String(IMU_Data[5],2)+"\t");
        Serial.print("Mag XYZ: ");  
        Serial.print(String(IMU_Data[6],1)+"\t"+String(IMU_Data[7],1)+"\t"+String(IMU_Data[8],1)+"\t");
        //Serial.print("Temp °C: ");        
        //Serial.print(String(IMU_Data[9],1)+"\t");
        Serial.println("");
      }*/
  }

  // ------------------------------------------------------------------------------------------------- //
  // ------------------------------------------- Functions ------------------------------------------- //
  // ------------------------------------------------------------------------------------------------- //
      
    void    IMU_Get_Data(float * dest){
    I2C_Read_Byte(MPU_I2C_ADD,0x3B,14,I2C_buffer);  // Read MPU acc, temp, gyro

    dest[0] = (float)((int16_t)(I2C_buffer[ 0]<<8 | I2C_buffer[ 1]))*16.0/32768.0;   // Acc X
    dest[1] = (float)((int16_t)(I2C_buffer[ 2]<<8 | I2C_buffer[ 3]))*16.0/32768.0;   // Acc Y
    dest[2] = (float)((int16_t)(I2C_buffer[ 4]<<8 | I2C_buffer[ 5]))*16.0/32768.0;   // Acc Z

    dest[0] = (dest[0] * aSF[0]) + aBias[0];    // Correct scale factor then zero
    dest[1] = (dest[1] * aSF[1]) + aBias[1];
    dest[2] = (dest[2] * aSF[2]) + aBias[2];
    
    dest[9] = (float)((int16_t)(I2C_buffer[ 6]<<8 | I2C_buffer[ 7]))/333.87 + 21.0;  // Temp C
    
    dest[3] = (float)((int16_t)(I2C_buffer[ 8]<<8 | I2C_buffer[ 9]))*16.0/32768.0;   // Gyr X
    dest[4] = (float)((int16_t)(I2C_buffer[10]<<8 | I2C_buffer[11]))*16.0/32768.0;   // Gyr Y
    dest[5] = (float)((int16_t)(I2C_buffer[12]<<8 | I2C_buffer[13]))*16.0/32768.0;   // Gyr Z

    dest[3] -= gBias[0];   // Correct for drift
    dest[4] -= gBias[1];
    dest[5] -= gBias[2];

    I2C_Read_Byte(AK8_I2C_ADD,0x03,7,I2C_buffer);  // Read magnetometer

    dest[6] = (float)((int16_t)(I2C_buffer[ 1]<<8 | I2C_buffer[ 0]))*10.0*4912.0/32760.0*mScaleX;   // Mag X
    dest[7] = (float)((int16_t)(I2C_buffer[ 3]<<8 | I2C_buffer[ 2]))*10.0*4912.0/32760.0*mScaleY;   // Mag Y
    dest[8] = (float)((int16_t)(I2C_buffer[ 5]<<8 | I2C_buffer[ 4]))*10.0*4912.0/32760.0*mScaleZ;   // Mag Z

    dest[6] -= mBias[0];    // Correct for magnetic bias
    dest[7] -= mBias[1];
    dest[8] -= mBias[2];
  }
    void    IMU_Get_Data_Average(float * dest, int SumNo){
      
      float Sums[10]={};   // tempoary store
      
      for(int i=0; i<SumNo; i++){   // Get avrage of n readings
        IMU_Get_Data(IMU_Data);       // Get IMU data and put into array
        
        for(int j=0; j<10; j++){
          Sums[j] = Sums[j] + IMU_Data[j];    // Sum
        }
      }
      
      for(int i = 0; i<10; i++){
        dest[i] = Sums[i]/(float)SumNo;    // find average
      }
      
    }
    int     IMU_Begin(int Serial_Debug){    
      // check if serial printing is allowed and if it isnt, dont print
        if(SerialPrint==0){Serial_Debug = 0;}
      
      // Writing all registers
        if(Serial_Debug){Serial.println("");} 
        if(Serial_Debug){Serial.println("Resetting MPU");}                I2C_Write(MPU_I2C_ADD,0x6B,0x80);                 // Reset MPU
        delay(50);
        if(Serial_Debug){Serial.println("1: Checking if MPU responds");}  if(MPU_whoAmI()!= 0x71             ){return  1;}  // Check if the MPU responds with its address
        if(Serial_Debug){Serial.println("2: Setting SRD");}               if(I2C_Write(MPU_I2C_ADD,0x19,0x80)){return  2;}  // Set sample rate dividor = 0
        if(Serial_Debug){Serial.println("3: Setting gyro bandwidth");}    if(I2C_Write(MPU_I2C_ADD,0x1A,0x04)){return  3;}  // Set gyr 20Hz bandwidth
        if(Serial_Debug){Serial.println("4: Setting gyro range");}        if(I2C_Write(MPU_I2C_ADD,0x1B,0x1B)){return  4;}  // Set to +-2000degs-1 range
        if(Serial_Debug){Serial.println("5: Setting accel range");}       if(I2C_Write(MPU_I2C_ADD,0x1C,0x18)){return  5;}  // Set to +-16G range      
        if(Serial_Debug){Serial.println("6: Setting accel bandwidth");}   if(I2C_Write(MPU_I2C_ADD,0x1D,0x0C)){return  6;}  // Set acc 20Hz bandwidth
        if(Serial_Debug){Serial.println("7: Bypassing I2C");}             if(I2C_Write(MPU_I2C_ADD,0x37,0x02)){return  7;}  // Bypass I2C by turning off I2C master mode
        delay(50);
        if(Serial_Debug){Serial.println("8: Resetting AK8963");}          if(I2C_Write(AK8_I2C_ADD,0x0A,0x00)){return  8;}  // Reset AK8963
        delay(50);
        if(Serial_Debug){Serial.println("9: Checking if AK responds");}   if(AK8963_whoAmI()!= 0x48          ){return  9;}  // Check if AK8 responds with its address
        if(Serial_Debug){Serial.println("10:Resetting AK8963");}          if(I2C_Write(AK8_I2C_ADD,0x0A,0x00)){return 10;}  // Reset AK8963
        delay(50);
        if(Serial_Debug){Serial.print("11:Enter FUSE rom mode... ");}     if(I2C_Write(AK8_I2C_ADD,0x0A,0x0F)){return 11;}  // Enter FUSE rom mode
        delay(50);
        if(Serial_Debug){Serial.print("Reading rom... ");} 
        I2C_Read_Byte(AK8_I2C_ADD,0x10, 3, I2C_buffer);                // Read sensetivity adjustment values      
        mScaleX = ((((float)I2C_buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla per LSB
        mScaleY = ((((float)I2C_buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla per LSB
        mScaleZ = ((((float)I2C_buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla per LSB
        if(Serial_Debug){Serial.println("Good");} 
        if(Serial_Debug){Serial.println("12:Resetting AK8963");}          if( I2C_Write(AK8_I2C_ADD,0x0A,0x00)<0 ){return 12;}  // Reset AK8963
        delay(50);
        if(Serial_Debug){Serial.println("13:Set to 16 bit, 100Hz");}      if( I2C_Write(AK8_I2C_ADD,0x0A,0x16)<0 ){return 13;}  // Set to 16 bit, 100Hz update rate
        delay(50);
  
        return 0;   // Returns 0 if works
    }
    void    I2C_Scan(int Serial_Debug){
      // check if serial printing is allowed and if it isnt, dont print
        if(SerialPrint==0){Serial_Debug = 0;}

      byte error, address;
        if(SerialPrint){Serial.println("Scanning I2C...");}
        int nDevices=0;
        
        for(address = 1; address < 127; address++){
          Wire.beginTransmission(address);
          error = Wire.endTransmission();
          
          if (error == 0) {
            if(Serial_Debug){Serial.print("I2C dev at: 0x");}
            if(address < 16){if(Serial_Debug){Serial.print("0");}}  // If address below 16, print 0
            if(Serial_Debug){Serial.println(address, HEX);}         // Print hex value      
            nDevices++;
          }
          
          if (error == 4) {
            if(Serial_Debug){Serial.print("Error at dev: 0x");}
            if(address < 16){if(Serial_Debug){Serial.print("0");}}  // If address below 16, print 0
            if(Serial_Debug){Serial.println(address, HEX);}         // Print hex value      
          }    
        }
        
      if (nDevices == 0){if(Serial_Debug){Serial.println("No devices found");}}      
    }
    int     I2C_Write(uint8_t devaddress, uint8_t subaddress, uint8_t data){
      Wire.beginTransmission(devaddress);   // Transmit to target device
      Wire.write(subaddress);               // Specify which subaddress
      Wire.write(data);                     // Write data
      Wire.endTransmission();               // End transmission   
      
      I2C_Read_Byte(devaddress,subaddress,1,I2C_buffer);    // Read what was written
      
      if(I2C_buffer[0] == data){
        return 0;                           // If correct, output 0
      }else{
        return -1;                          // If wrong, output -1
      }  
    }
    void    I2C_Read_Byte(uint8_t devaddress, uint8_t subaddress, uint8_t count, uint8_t * dest){
      Wire.beginTransmission(devaddress);         // Transmit to target device
      Wire.write(subaddress);                     // Specify what subaddress
      Wire.endTransmission(false);                // Leave I2C lines online
      uint8_t i = 0;
      Wire.requestFrom(devaddress,count);         // Request bytes
      while(Wire.available()){dest[i++] = Wire.read();}
    }
    uint8_t MPU_whoAmI(){
       I2C_Read_Byte(MPU_I2C_ADD, 0x75,1,I2C_buffer);
      return I2C_buffer[0];
    }
    uint8_t AK8963_whoAmI(){
      I2C_Read_Byte(AK8_I2C_ADD, 0x00,1,I2C_buffer);
      return I2C_buffer[0];
    }    
    void    IMU_Zero_Acc(float * dest){
      float aXSum, aYSum, aZSum;
    
      for(int i = 0; i<50; i++){
        IMU_Get_Data(IMU_Data);   // Get IMU data and put into array
  
        aXSum = aXSum+IMU_Data[0];
        aYSum = aYSum+IMU_Data[1];
        aZSum = aZSum+IMU_Data[2];
      }

      aXSum /= 50;
      aYSum /= 50;
      aZSum /= 50;
      
      aBias[0] = -aXSum;
      aBias[1] = -aYSum;
      aBias[2] = 1.0-aZSum;      
    }
    void    IMU_Zero_Gyr(float * dest){
      float gXSum, gYSum, gZSum;
    
      for(int i = 0; i<50; i++){
        IMU_Get_Data(IMU_Data);   // Get IMU data and put into array
  
        gXSum = gXSum+IMU_Data[3];
        gYSum = gYSum+IMU_Data[4];
        gZSum = gZSum+IMU_Data[5];
      }

      gBias[0] = gXSum/50;
      gBias[1] = gYSum/50;
      gBias[2] = gZSum/50;
      
    }
    void    Mad_Update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){
      float q1 = qMad[0], q2 = qMad[1], q3 = qMad[2], q4 = qMad[3];   // short name local variable for readability
      float norm;
      float hx, hy, _2bx, _2bz;
      float s1, s2, s3, s4;
      float qDot1, qDot2, qDot3, qDot4;

      // Auxiliary variables to avoid repeated arithmetic
        float _2q1mx;        float _2q1my;
        float _2q1mz;        float _2q2mx;
        float _4bx;          float _4bz;
        float _2q1 = 2.0f * q1;        
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;        
        float _2q4 = 2.0f * q4;
        float _2q1q3 = 2.0f * q1 * q3;        
        float _2q3q4 = 2.0f * q3 * q4;
        float q1q1 = q1 * q1;        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;        float q4q4 = q4 * q4;

      // Normalise accelerometer measurement
        norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f/norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

      // Normalise magnetometer measurement
        norm = sqrtf(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f/norm;
        mx *= norm;
        my *= norm;
        mz *= norm;

      // Reference direction of Earth's magnetic field
        _2q1mx = 2.0f * q1 * mx;
        _2q1my = 2.0f * q1 * my;
        _2q1mz = 2.0f * q1 * mz;
        _2q2mx = 2.0f * q2 * mx;
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

      // Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        norm = 1.0f/norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

      // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

      // Integrate to yield quaternion
        q1 += qDot1 * deltat;
        q2 += qDot2 * deltat;
        q3 += qDot3 * deltat;
        q4 += qDot4 * deltat;
        norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        norm = 1.0f/norm;
        qMad[0] = q1 * norm;
        qMad[1] = q2 * norm;
        qMad[2] = q3 * norm;
        qMad[3] = q4 * norm;
      }
    void    Mah_Update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){
      float q1 = qMah[0], q2 = qMah[1], q3 = qMah[2], q4 = qMah[3];   // short name local variable for readability
      float norm;
      float hx, hy, bx, bz;
      float vx, vy, vz, wx, wy, wz;
      float ex, ey, ez;
      float pa, pb, pc;

      // Auxiliary variables to avoid repeated arithmetic
        float q1q1 = q1 * q1;      float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;      float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;      float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;      float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;      float q4q4 = q4 * q4;   

      // Normalise accelerometer measurement
        norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

      // Normalise magnetometer measurement
        norm = sqrtf(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;        // use reciprocal for division
        mx *= norm;
        my *= norm;
        mz *= norm;

      // Reference direction of Earth's magnetic field
        hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
        hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
        bx = sqrtf((hx * hx) + (hy * hy));
        bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

      // Estimated direction of gravity and magnetic field
        vx = 2.0f * (q2q4 - q1q3);
        vy = 2.0f * (q1q2 + q3q4);
        vz = q1q1 - q2q2 - q3q3 + q4q4;
        wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
        wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
        wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

      // Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
      
        if (Ki > 0.0f){
          eInt[0] += ex;      // accumulate integral error
          eInt[1] += ey;
          eInt[2] += ez;
        }else{
          eInt[0] = 0.0f;     // prevent integral wind up
          eInt[1] = 0.0f;
          eInt[2] = 0.0f;
        }

      // Apply feedback terms
        gx = gx + Kp * ex + Ki * eInt[0];
        gy = gy + Kp * ey + Ki * eInt[1];
        gz = gz + Kp * ez + Ki * eInt[2];

      // Integrate rate of change of quaternion
        pa = q2;
        pb = q3;
        pc = q4;
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

      // Normalise quaternion
        norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        norm = 1.0f / norm;
        qMah[0] = q1 * norm;
        qMah[1] = q2 * norm;
        qMah[2] = q3 * norm;
        qMah[3] = q4 * norm;
  }
    void    AHRS_Update(float q1, float q2, float q3, float q4, float * Euler_Angles){
      Euler_Angles[0]  = atan2(2.0f * (q2*q3 + q1*q4), q1*q1 + q2*q2 - q3*q3 - q4*q4);   
      Euler_Angles[1]  = -asin(2.0f * (q2*q4 - q1*q3));
      Euler_Angles[2]  = atan2(2.0f * (q1*q2 + q3*q4), q1*q1 - q2*q2 - q3*q3 + q4*q4);
      Euler_Angles[1] *= 180.0f / PI;
      Euler_Angles[0] *= 180.0f / PI; 
      Euler_Angles[0] -= MagDeclination;
      Euler_Angles[2] *= 180.0f / PI;
    }
    
    
    
    
    

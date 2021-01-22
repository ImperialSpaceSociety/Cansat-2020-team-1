
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

      float mBias[3] = {32.0, 15.0, -77.0};           // Mag bias 
      float mSF[3]   = {1.00000, 1.00000, 1.00000};   // Scale factor for mag       
      float aSF[3]   = {0.98861, 0.98924, 0.98900};   // Scale factor for acc
      #define MagDeclination 0.0                      // Magnetic declination

  // ------------------------------------------------------------------------------------------------- //
  // --------------------------------------------- Setup --------------------------------------------- //
  // ------------------------------------------------------------------------------------------------- //
  
  void setup() {
    // Setup serial
      Serial.begin(115200);      
      
    // Setup I2C
      Wire.setSDA(PB9);
      Wire.setSCL(PB8);
      Wire.begin();
      
    // Scan I2C lines
      I2C_Scan(0);      // perform I2C scan: 1 = debug printing, 0 = no printing
      
    // Start IMU
      //if(SerialPrint){Serial.print("MPU9250... ");}
      int IMU_status = IMU_Begin(0);                      // Start IMU: 1 = debug printing, 0 = no printing
      if(IMU_status){                                     // if any errors, stop program
        //if(SerialPrint){Serial.print("ERROR ID: "+String(IMU_status));}
        while(1){};                                                       
      }
      //Serial.println("Good");      
      IMU_Zero_Acc(aBias);    // Zero accelerometer
      IMU_Zero_Gyr(gBias);    // Zero gyroscope
    // Other stuff
      //delay(100);
  }

  // ------------------------------------------------------------------------------------------------- //
  // --------------------------------------------- Loop  --------------------------------------------- //
  // ------------------------------------------------------------------------------------------------- //

  void loop() {

    // IMU
      IMU_Get_Data(IMU_Data);

      Serial.flush(); 
      Serial.print(IMU_Data[6]); 
      Serial.print(",");
      Serial.print(IMU_Data[7]);
      Serial.print(",");
      Serial.print(IMU_Data[8]);
      Serial.println();
      delay(50);
  }

  // ------------------------------------------------------------------------------------------------- //
  // ------------------------------------------- Functions ------------------------------------------- //
  // ------------------------------------------------------------------------------------------------- //
      
    void IMU_Get_Data(float * dest){
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

    dest[6] = dest[6]*mSF[0] - mBias[0];    // Correct for scale factor then zero
    dest[7] = dest[7]*mSF[1] - mBias[1];
    dest[8] = dest[8]*mSF[2] - mBias[2];
  }
    void IMU_Get_Data_Average(float * dest, int SumNo){
      
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
    int  IMU_Begin(int Serial_Debug){    
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
    void I2C_Scan(int Serial_Debug){
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
    int  I2C_Write(uint8_t devaddress, uint8_t subaddress, uint8_t data){
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
    void I2C_Read_Byte(uint8_t devaddress, uint8_t subaddress, uint8_t count, uint8_t * dest){
      Wire.beginTransmission(devaddress);         // Transmit to target device
      Wire.write(subaddress);                     // Specify what subaddress
      Wire.endTransmission(false);                // Leave I2C lines online
      uint8_t i = 0;
      Wire.requestFrom(devaddress,count);         // Request bytes
      while(Wire.available()){dest[i++] = Wire.read();}
    }
    byte MPU_whoAmI(){
       I2C_Read_Byte(MPU_I2C_ADD, 0x75,1,I2C_buffer);
      return I2C_buffer[0];
    }
    byte AK8963_whoAmI(){
      I2C_Read_Byte(AK8_I2C_ADD, 0x00,1,I2C_buffer);
      return I2C_buffer[0];
    }    
    void IMU_Zero_Acc(float * dest){
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
    void IMU_Zero_Gyr(float * dest){
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



    

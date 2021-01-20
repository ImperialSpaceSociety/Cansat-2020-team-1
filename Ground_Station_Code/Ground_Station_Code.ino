
  // For RGB Led
    #define LED_R 6
    #define LED_G 5
    #define LED_B 3
  // For LORA
  
  void setup() {
    // RGB led channels
      pinMode(LED_R, OUTPUT);
      pinMode(LED_G, OUTPUT);
      pinMode(LED_B, OUTPUT);
    // Serial
      Serial.begin(115200);                   // Begin Serial
      while(!Serial){}                        // Wait for serial to open
      Serial.println("Ground station start");
  }
  
  void loop() {
    
    for(int i = 0; i<255; i++){
      RGB_Led(  i,  0,  0);
      delay(10);
    }
    for(int i = 0; i<255; i++){
      RGB_Led(255-i,  0,  0);
      delay(10);
    }
    
    for(int i = 0; i<255; i++){
      RGB_Led(  0,  i,  0);
      delay(10);
    }
    for(int i = 0; i<255; i++){
      RGB_Led(  0,  255-i,  0);
      delay(10);
    }
    
    for(int i = 0; i<255; i++){
      RGB_Led(  0,  0,  i);
      delay(10);
    }
    for(int i = 0; i<255; i++){
      RGB_Led(  0,  0,  255-i);
      delay(10);
    }
    
  }

  void RGB_Led(uint8_t R, uint8_t G, uint8_t B){
    R = R/2;
    G = G/2;
    B = B/2;
    analogWrite(LED_R,R);
    analogWrite(LED_G,G);
    analogWrite(LED_B,B);
  }

/*
 Textile Pressure Sensor (16 x 16 Matrix) front-end
  Multiplexing with current sensing and communication
 
 Running Node.js processes asynchronously using serial port
 
  
 created 15 Dec 2014
 by Jose F. Saenz C.
 
 */

/* ************************************** 
  Definitions
*/

// Libriries
#include "SPI.h"
#include "EEPROM.h"
#include <avr/pgmspace.h>

// Hardware settings
#define dacMinValue 2048
#define dacMaxValue 4095
#define LED13 13
#define DAC_CS 11  //Chip select pin for DAC
#define MuxCol_SYNC 12 //SYNC  pin for column Mux
#define MuxRow_SYNC 10 //SYNC  pin for row Mux
#define PGA_CS 9 //Chip select pin for PGA
#define ADC_CS 8 //Chip select pin for ADC
#define analogIn 0 // Analog input 0 for current sensing reading. 
#define linuxBaud 250000
#define usbBaud 115200

//Program settings
#define endFrameByte  10 //ASCII enter byte for ending frames
#define stopServerByte 251 //
byte rows = 32;
byte columns = 32;
int dimension = 1024;
byte equilibrationThreshold = 247; //if a pressure of 247kPa is applied (i.e. 200kg over 81cm2) is convenient to set the digital output in the same scale
byte acquisitionArea[4] = {0, 0, 32, 32};

// Global varaibles definition 
byte state;    //State variable
byte bytes2send = 1;
byte row, col; //Matrix activation variables
int n, f; //sensor index and frame variable
byte FrameByte1[1024];
byte Gpga = 1;  //Output gain to set on PGA (1 = x2)
bool eqFlag = false;
bool debugFlag = false;
bool areaFlag = false;
byte framesSent = 0;

// Equilibration data:
//Matrix 32x32_1
const byte eqFrame[] PROGMEM = {9,23,11,13,9,17,18,12,25,16,36,12,28,18,31,28,34,21,20,30,32,8,16,16,22,15,15,14,17,18,15,8,10,12,15,18,7,14,18,11,21,20,14,12,28,25,26,17,22,13,20,9,6,13,8,6,7,18,10,18,14,10,20,12,16,16,18,13,14,12,9,10,12,15,25,13,22,21,18,12,24,15,13,15,14,13,12,15,14,15,12,13,7,4,5,14,15,18,11,16,14,7,16,13,16,14,15,11,17,17,17,16,11,13,19,14,12,16,9,25,10,11,11,8,5,13,10,16,26,28,16,31,23,15,15,20,23,19,17,17,27,30,15,17,17,18,26,18,12,15,15,12,12,13,11,12,8,14,11,21,19,22,14,15,22,24,17,12,15,15,13,21,11,20,11,9,12,14,6,12,14,9,7,6,7,14,15,17,9,13,8,18,11,11,8,11,17,25,14,16,14,24,12,15,22,18,21,35,44,22,19,9,8,13,12,10,11,8,13,10,8,6,11,13,14,10,10,10,8,15,12,13,13,11,13,12,13,14,12,17,16,16,14,9,11,12,8,15,10,11,9,9,9,11,11,8,12,3,11,11,9,8,13,10,8,12,8,8,13,12,8,14,13,30,13,11,11,8,9,12,16,13,9,9,5,6,7,10,11,10,13,8,10,9,10,11,8,11,7,11,11,14,12,12,11,10,11,7,9,15,6,8,8,11,9,9,12,8,10,11,21,13,9,11,17,14,11,12,10,10,8,7,8,21,18,14,25,17,9,13,9,7,16,10,7,12,14,10,12,6,12,8,13,9,8,14,14,13,14,13,8,7,4,9,19,9,11,13,13,15,10,11,14,14,10,16,11,7,8,11,7,9,8,7,36,11,9,14,17,14,14,12,10,18,22,17,13,21,14,14,14,16,12,11,11,13,13,10,11,10,11,8,11,6,9,14,16,10,9,26,13,15,13,11,9,6,18,12,14,24,34,15,13,21,16,13,6,9,8,11,10,6,7,7,8,5,9,10,20,20,15,23,14,17,11,11,10,9,5,12,7,12,10,11,18,32,14,14,11,14,11,10,16,9,10,6,7,4,9,8,20,26,13,22,21,20,18,13,15,18,20,20,10,22,24,17,10,14,15,15,15,11,10,12,13,12,10,0,8,10,12,9,21,13,19,11,18,15,16,14,9,20,9,14,6,14,9,14,10,23,21,15,11,15,13,11,8,15,13,10,6,8,10,10,15,22,19,17,24,14,26,13,14,20,17,26,21,24,24,25,19,18,14,16,13,21,16,18,14,12,10,6,9,9,10,11,15,12,21,12,18,15,14,16,16,15,15,22,15,15,14,16,18,23,12,18,15,15,21,12,20,12,15,11,10,10,10,12,17,11,10,11,12,8,8,20,10,9,18,26,14,22,21,13,22,25,23,22,18,18,29,22,12,15,16,11,13,8,12,10,22,18,15,12,28,17,14,15,12,18,18,19,9,24,13,22,26,20,18,14,11,9,12,11,8,11,14,12,8,7,7,8,15,16,13,10,10,9,12,11,14,11,13,12,13,12,9,18,16,16,8,14,13,10,12,9,10,8,10,11,10,7,8,7,21,16,11,12,12,13,9,9,13,13,13,13,13,11,12,13,12,16,18,12,8,9,12,12,12,12,8,8,12,8,9,7,15,17,10,12,13,12,15,10,12,11,9,18,10,12,11,11,11,14,11,10,9,12,12,10,10,12,9,11,9,9,8,8,17,16,10,13,14,9,9,10,8,14,7,11,11,13,13,27,19,15,10,10,12,14,15,19,11,9,9,11,11,6,10,6,19,14,9,10,13,9,9,12,10,7,12,15,8,12,6,13,13,12,20,7,8,13,8,10,10,7,5,6,7,6,3,5,13,19,10,9,8,7,8,11,8,6,7,13,6,9,13,10,13,17,6,8,5,9,7,5,7,7,9,8,8,7,7,5,28,25,11,14,14,7,9,8,7,12,10,16,7,10,15,15,17,13,9,9,10,8,10,12,10,8,9,6,7,5,6,9,15,17,13,14,15,11,12,9,7,15,10,10,7,8,11,13,9,13,22,16,9,9,10,9,12,8,13,13,8,8,7,8,0,14,27,16,16,24,19,14,9,13,14,9,8,21,22,13,15,16,14,18,7,7,9,10,8,12,14,6,6,7,5,6,15,25,20,14,9,12,10,9,11,16,10,12,14,22,14,15,8,17,17,14,9,7,12,14,11,9,9,10,11,8,16,26,19,19,14,20,16,14,11,14,19,13,8,8,10,18,14,24,9,15,12,10,9,6,10,10,14,16,14,16,9,13,15,7};

/***************************************************************************** 
  Initialization routine
******************************************************************************/ 
void setup() {
  // initialize digital pins:
  pinMode(DAC_CS, OUTPUT);
  pinMode(MuxCol_SYNC, OUTPUT);
  pinMode(MuxRow_SYNC, OUTPUT);  
  pinMode(PGA_CS, OUTPUT);
  pinMode(ADC_CS, OUTPUT);  
  digitalWrite(DAC_CS, HIGH);
  digitalWrite(MuxCol_SYNC, HIGH);
  digitalWrite(MuxRow_SYNC, HIGH);  
  digitalWrite(PGA_CS, HIGH);
  digitalWrite(ADC_CS, HIGH);
  pinMode(LED13, OUTPUT);
  
  // initialize analog pins:
  analogReference(EXTERNAL);//
  
  // initialize state variables
  InitVariables();
  
  // initialize hardware links  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
    
  // initialize comunication modules  
  Serial.begin(usbBaud);   // Initialize the Serial (newline on)   
  //while (!Serial);  // Wait until a Serial Monitor is connected.
  Serial1.begin(linuxBaud);  // open serial connection to Linux    
  digitalWrite(LED13, HIGH); //turn led on to indicate that arduino is ready
  Serial.println("Arduino Ready");
  delay(1000);
  digitalWrite(LED13, LOW); //turn led off until Linux is ready
}

/***************************************************************************** 
  Main loop
******************************************************************************/  
void loop() {  
  if (state==0){  // Get commands if idle
    GetLinuxCommand();
    GetUSBCommand(); 
    if (eqFlag){
      Serial.println("Equilibrating...");
      //EquilibrateSensors(Gpga); 
      Serial1.println("#");//
       Serial1.println("#");// 
      Serial.println("equilibration done");
      eqFlag = false;
    }
  } else if (state>0){   // If in the active state activate the corresponding sensor element, read, store and transmits data... 
         
    // set test voltage of corresponding sensor n (DAC setting)
    byte Vdac = EEPROM.read(row*32 + col);//equilibrationThreshold;//24//64
    if (Vdac == 255) Vdac = 24;
    SetTestVoltage(Vdac);  
    // set multiplexers
    SetMultiplexers(row, col);   
    // set reading gain for corresponding sensor n (PGA setting)
    SetReadingGain(Gpga);//Gpga         
    // read sensor (external or internal ADC)
    int adcRead = GetAnalogRead();//analogRead(analogIn);//GetAnalogRead();
    
    // format analog reading for ASCII compatible comunication
    FrameByte1[n] = FormatAnalogByte1(adcRead);
       
    // Once all sensor elements have been read transmits data to node
    if (n == dimension-1){   
      //log frame read time
      int tim = timer(true);      
      Serial.print(" f read:");
      Serial.println(tim);
      if (!debugFlag){
        if (tim<10){
          delay(10-tim);
        } 
      } else{
        delay(1000);
      }   
      // check for frame received confirmation
      //CheckLastDataSending();      
      if (state > 0 && framesSent<50){   
        for (int j = 0; j <= n; j++){
          Serial1.write(FrameByte1[j]);
        }            
        int tframe=timer(true);
        Serial.print(" f sent:");
        if (tframe == 127) tframe = 128;
        Serial.println(tframe);           
        Serial1.write(tframe);//Sent frame time   
        Serial1.write(endFrameByte);//Sent end signal 
        framesSent++;    
        timer(false);//restart timer 
      } else{
        Serial1.println("'");//remind that acquisition is goind and ask for an acknowledge
      }              
      n=0;//starts form sensor 0 
      col = acquisitionArea[0];
      row = acquisitionArea[1]; 
      
      // check incoming data or commands from Node
      GetLinuxCommand();
      GetUSBCommand();
      if (timer(true)>2000 && !debugFlag) {
        InitVariables();
        Serial1.println("#");// 
        Serial.println("Data overflow");
      }
    } else {
      n++;    
      col++;
      if (col >= acquisitionArea[2]) {
        col = acquisitionArea[0];
        row++;
        if (row >= acquisitionArea[3]) {
          row = acquisitionArea[1];
        }
      }
    }
  } 
}

/***************************************************************************** 
  Function:  timer
******************************************************************************/ 
unsigned long timer(boolean toc){
  static unsigned long time0=0;
  if (toc){
    //unsigned long time=millis()-time0;
    return millis()-time0;
  } else{
    time0=millis();
    return 0;
  }
}

/***************************************************************************** 
  Function:  GetLinuxCommand
******************************************************************************/ 
void GetLinuxCommand(){
  static byte commandMode = 0; //indication of command arriving from Node  
  while (Serial1.available()>0) {
    int nodeoutput = Serial1.read();
    if (state<1){
      Serial.write(nodeoutput);// pass any incoming bytes from the node process to the serial port
    }
    if (nodeoutput == '!'){//stop signal
      InitVariables();
      Serial1.println("#");
      Serial.println("State 0"); 
    } else if (nodeoutput == '%'){
      Serial.print(" frame ok: ");
           
      if (framesSent==1){
        framesSent--;
      }else if (framesSent>=2){
        framesSent = framesSent-2; 
      }else{
        framesSent =0;
      } 
      Serial.print(framesSent);
      Serial.print(" remaining. ");
      if (state==3){
        state=1;
      }
    }
    if (commandMode == 0 && nodeoutput == '$') {                 //   '$' key pressed?
      commandMode = 1;           //       enter in command mode 1
    } else if (commandMode == 1 && nodeoutput == '~'){
      commandMode = 2;           //       enter in command mode 2 "command"
    } else if (commandMode == 1 && nodeoutput == '#'){
      commandMode = 3;           //       enter in command mode 3 "data setting"
    } else if (commandMode == 2){                          // if we are in command mode...
      switch (nodeoutput) {
        case '1': //Start acquisition command
          if (state==0){
            state=1;         
            Serial1.println("$");
            Serial.println("State 1");
            delay(200);
            while (Serial1.available()>0) {
              Serial.write(Serial1.read());
            }
            Serial.println("\n start acquisition");
            timer(false);//restart timer
            framesSent = 0;
          }
          break;
        case '2': //Equilibration command
          if (state==0){
            eqFlag=true;
            state=2;
            Serial.println("Equilibrating...");  
          }        
          break; 
        case '3':
          if (state==0){
            Serial1.write(35+state);
            Serial1.write(endFrameByte);//Sent end signal 
            while (Serial1.available()>0) {
              Serial.write(Serial1.read());
            }
            Serial.print("Equilibration frame request...");
            int dato;            
            for (int n = 0; n < 1024; n++){
              dato = EEPROM.read(n) + 40;
              if (dato > 255) dato = 255;
              Serial1.write(dato);
              Serial.print("->edato ");
              Serial.print(n);
              Serial.print(": ");
              Serial.println(dato);
            }
            Serial1.write(endFrameByte);//Sent end signal
          }          
          break;
        case '4': //Bad frame command
          Serial.print(" frame not ok");
          if (state==3){
            state=1;
          }
          break;
        case '5': //get State command
          Serial1.write(35+state);
          Serial1.write(endFrameByte);//Sent end signal 
          break;
        case '6':
          if (state==0){
            byte eqData;
            for (int n = 0; n < 1024; n++){
              Serial.print("Sensor: ");
              Serial.print(n);
              Serial.print(", eq value: ");
              eqData = pgm_read_byte_near(eqFrame + n);
              Serial.println(eqData);
              //EEPROM.write(n, eqData);
            }
          }
          break;
        case '7': //Area setting command
          if (state==0){
            areaFlag=true;
            Serial.println("New acquisition area..."); 
            columns = acquisitionArea[2] - acquisitionArea[0];
            rows = acquisitionArea[3] - acquisitionArea[1];
            dimension = rows*columns;
          }        
          break; 
        case '$': //Linux ready command
          if (state==0) {
            Serial1.println(" ");
            delay(1000);
            Serial1.println("node /mnt/sda1/toymat/index.js 2> /mnt/sda1/toymat/error.log"); 
            Serial.println("Node process launched");
            digitalWrite(13, HIGH);
          }
          break;
      }
      commandMode = 0;           //       exit command mode      
    } else if (commandMode == 3){ 
      char digits[4];
      byte bytes;
      switch (nodeoutput) {
        case 'r': //single byte data
          bytes = Serial1.readBytes(digits, 2);
          if (bytes == 2){
            if (digits[1]<48){
              rows = byte(chars2Int(digits[0],'0','0'));
            } else{
              rows = byte(chars2Int(digits[1],digits[0],'0'));
            }
            Serial.print("rows: ");
            Serial.println(rows);
          } else {
            Serial.println("error reading row value");  
          }       
          break;
        case 'c': //double byte data
          bytes = Serial1.readBytes(digits, 2);
          if (bytes == 2){
            if (digits[1]<48){
              columns = byte(chars2Int(digits[0],'0','0'));
            } else{
              columns = byte(chars2Int(digits[1],digits[0],'0'));
            }
            Serial.print("columns: ");
            Serial.println(columns);
          } else Serial.println("error reading col value"); 
          break;
        case 'b': //double byte data
          bytes = Serial1.readBytes(digits, 1);
          if (bytes == 1){
            bytes2send = byte(chars2Int(digits[0],'0','0'));
            Serial.print("bytes2send: ");
            Serial.println(bytes2send);
          } else Serial.println("error reading bytes2send value"); 
          break;
        case 'g': //double byte data
          bytes = Serial1.readBytes(digits, 1);
          if (bytes == 1){
            Gpga = byte(chars2Int(digits[0],'0','0'));
            Serial.print("Gpga: ");
            Serial.println(Gpga);
          } else Serial.println("error reading gain value"); 
          break;
        case 'e': //double byte data
          bytes = Serial1.readBytes(digits, 3);
          if (bytes == 3){
            if (digits[2]<48){
              equilibrationThreshold = byte(chars2Int(digits[1],digits[0],'0'));
            } else{
              equilibrationThreshold = byte(chars2Int(digits[2],digits[1],digits[0]));
            }            
            Serial.print("equilibrationThreshold: ");
            Serial.println(equilibrationThreshold);
          } else Serial.println("error reading eqTh value"); 
          break;
        case 'x': //double byte data
          bytes = Serial1.readBytes(digits, 2);
          if (bytes == 2){
            if (digits[1]<48){
              acquisitionArea[0] = byte(chars2Int(digits[0],'0','0'));
            } else{
              acquisitionArea[0] = byte(chars2Int(digits[1],digits[0],'0'));
            }
            Serial.print("X0: ");
            Serial.println(acquisitionArea[0]);
          } else Serial.println("error reading X0 value"); 
          break;
        case 'y': //double byte data
          bytes = Serial1.readBytes(digits, 2);
          if (bytes == 2){
            if (digits[1]<48){
              acquisitionArea[1] = byte(chars2Int(digits[0],'0','0'));
            } else{
              acquisitionArea[1] = byte(chars2Int(digits[1],digits[0],'0'));
            }
            Serial.print("Y0: ");
            Serial.println(acquisitionArea[1]);
          } else Serial.println("error reading Y0 value"); 
          break;
        case 'X': //double byte data
          bytes = Serial1.readBytes(digits, 2);
          if (bytes == 2){
            if (digits[1]<48){
              acquisitionArea[2] = byte(chars2Int(digits[0],'0','0'));
            } else{
              acquisitionArea[2] = byte(chars2Int(digits[1],digits[0],'0'));
            }
            Serial.print("Xn: ");
            Serial.println(acquisitionArea[2]);
          } else Serial.println("error reading Xn value"); 
          break;
        case 'Y': //double byte data
          bytes = Serial1.readBytes(digits, 2);
          if (bytes == 2){
            if (digits[1]<48){
              acquisitionArea[3] = byte(chars2Int(digits[0],'0','0'));
            } else{
              acquisitionArea[3] = byte(chars2Int(digits[1],digits[0],'0'));
            }
            Serial.print("Yn: ");
            Serial.println(acquisitionArea[3]);
          } else Serial.println("error reading Yn value"); 
          break;
      }
      dimension = rows*columns;
      if (dimension>256){
        bytes2send=1;
      } else{
        bytes2send=2;
      }
      Serial.print("dimension: ");
      Serial.println(dimension);
      Serial.print("bytes2send: ");
      Serial.println(bytes2send);
      commandMode = 0;           //       exit command mode 
    }  
  }
}

/***************************************************************************** 
  Function:  GetUSBCommand
******************************************************************************/ 
void GetUSBCommand(){
  static boolean commandMode2 = false; //indication of command arriving from Node  
  while (Serial.available()>0) {
    int command = Serial.read();
    if (commandMode2 == false) {       // if we aren't in command mode...
      if (command == '$') {                 //    Dolar '$' key pressed?
        commandMode2 = true;           //       enter in command mode
      } else{
        Serial1.write(command);
      }      
    } else {                          // if we are in command mode...
      switch (command) {
        case '0':
          Serial1.println("!");//Stop Node
          InitVariables();
          Serial.println("State 0");
          break;
        case '1':
          if (state==0) {
             Serial1.println("node /mnt/sda1/toymat/index.js 2> /mnt/sda1/toymat/error.log"); 
             debugFlag = false;            
          }
          Serial.println("Node process launched");          
          break;
        case '2':
          if (state==0) {
             Serial1.println("node debug /mnt/sda1/toymat/index.js"); 
             debugFlag = true;
          }
          Serial.println("Node process launched");
          break;
        case '3':
          Serial1.write(3); 
           debugFlag = false; 
          //Serial1.write(3);
          Serial.println("Stop debug repl mode");
          break;
      }
      commandMode2 = false;           //       exit command mode  */    
    }    
  }
}

/***************************************************************************** 
  Function:  SetTestVoltage
******************************************************************************/ 
void SetTestVoltage(byte vdac){
  int vdacint = int(vdac<<3);
  int ddac = vdacint + 2048; //VDAC = Vdac*8 + 2048 (min=2.5V, max=5V)
  SPI.setDataMode(SPI_MODE0);   
  digitalWrite(DAC_CS, LOW);
  byte spidata = highByte(ddac); 
  spidata = 0b00001111 & spidata; //take the first four bits
  spidata = 0b00010000 | spidata; //add config bits: unbuffered, gain x2, Active operation (bit 12)
  SPI.transfer(spidata);
  spidata = lowByte(ddac);
  SPI.transfer(spidata);
  digitalWrite(DAC_CS, HIGH); 
}

/***************************************************************************** 
  Function:  SetReadingGain
******************************************************************************/ 
void SetReadingGain(byte gain){
  SPI.setDataMode(SPI_MODE0);
  digitalWrite(PGA_CS, LOW);
  SPI.transfer(0b01000000);
  SPI.transfer(gain);
  digitalWrite(PGA_CS, HIGH); 
}

/***************************************************************************** 
  Function:  SetMultiplexers
******************************************************************************/ 
void SetMultiplexers(byte r, byte c){
  SPI.setDataMode(SPI_MODE1);
  // set column mux
  digitalWrite(MuxCol_SYNC, LOW);
  SPI.transfer(c);
  digitalWrite(MuxCol_SYNC, HIGH); 
  // set row mux
  digitalWrite(MuxRow_SYNC, LOW);
  SPI.transfer(r);
  digitalWrite(MuxRow_SYNC, HIGH);
} 

/***************************************************************************** 
  Function:  GetAnalogRead
******************************************************************************/   
int GetAnalogRead(){
  SPI.setDataMode(SPI_MODE0);
  digitalWrite(ADC_CS, LOW);
  byte hspi = SPI.transfer(0);
  byte lspi = SPI.transfer(0);
  digitalWrite(ADC_CS, HIGH);
  int spidata = word(hspi, lspi);
  spidata = spidata >> 3; //Shift 3 position to eliminate redundant bits
  spidata &= 0x3FF; // and with 1023 to eliminate unknown bits after the 10bits data
  return spidata;
}

/***************************************************************************** 
  Function:  FormatAnalogByte1
******************************************************************************/   
byte FormatAnalogByte1(int adcRead){
  int data = (1023-adcRead) >> 2;
  data = data + 40 + 1;
  if (data==127)data=126;
  if (data>255){
    data=255;
  }else if (data<40){
    data=40;
  }
  return  byte(data); // make measure differential (2.5V=0) and 8 bits maximum (0V=255)  
}

/***************************************************************************** 
  Function:  FormatAnalogByte2
******************************************************************************/   
/*byte FormatAnalogByte2(int adcRead){
  adcRead = (1023-adcRead);
  adcRead = adcRead  & 0b0000000000000011;
  byte data = byte(adcRead) + 40; // make measure differential (2.5V=0) and 8 bits maximum (0V=255)  
  if (data>255){
    data=255;
  }else if (data<40){
    data=40;
  }
  return data;
} */

/***************************************************************************** 
  Function:  InitVariables
******************************************************************************/   
void InitVariables(){
  row = acquisitionArea[1];
  col = acquisitionArea[0];
  n=0;
  f=0;
  state=0;//intial state
} 

/***************************************************************************** 
  Function:  CheckLastDataSending
******************************************************************************/ 
void CheckLastDataSending(){
  GetLinuxCommand();
  // if previous frame has not been processed wait for confirmation
  int timeout=timer(true); 
  int tout=0;   
  Serial.println("waiting..."); //wait indication  
  while (state>=3){   
    //Serial.print("w"); //wait indication        
    if (timer(true)-timeout>100){ // wait for 10ms and then send end signal to reset comunication 
      //Serial1.write(endFrameByte);//Sent end signal, a new frame should be considered
      Serial.println("\n tout"); //timeout indication
      tout++;
      timeout=timer(true); 
      if (tout > 20){// if no confirmation received after 1s exit loop
        state=0;
        Serial1.write(endFrameByte);
        Serial.println("\ Node not responding, State 0");
      }
    }
   // check incoming data or commands from the node process 
    GetLinuxCommand();
    GetUSBCommand();                
  }
}

/***************************************************************************** 
  Function:  EquilibrateSensors
******************************************************************************/ 
void EquilibrateSensors(byte gain){
  byte iVdac=0;//Vs=2.5V
  int isensor;
  byte okpoints=0;
  byte maxVdacSensors=0;
  SetReadingGain(gain);//one gain for all
  for (int i = acquisitionArea[1]; i < acquisitionArea[3]; i++){
    for (int j = acquisitionArea[0]; j < acquisitionArea[2]; j++){
      isensor = i*32 + j;
      bool equilibratedFlag = false;
      GetUSBCommand();
      if (state!=2){
        return;
      }
      Serial.print("Sensor: ");
      Serial.print(isensor);
      while (!equilibratedFlag){
        SetTestVoltage(iVdac);
        SetMultiplexers(i, j);
        int adcRead = 1023-GetAnalogRead();
        if (adcRead >= equilibrationThreshold){
          okpoints++;
          if (okpoints>5){
            equilibratedFlag=true;
            Serial.print(" Position [");
            Serial.print(i);
            Serial.print(", ");
            Serial.print(j);
            Serial.print("] ok, iVdac: ");
            Serial.println(iVdac);
          }
          delay(10);//wait 10ms to confirn measure
        } else{
          iVdac++;
          if (iVdac >= 255){
            maxVdacSensors++;
            if (maxVdacSensors >= 200) {
              maxVdacSensors = 200;
            }
            equilibratedFlag=true;
            Serial.print(" Position [");
            Serial.print(i);
            Serial.print(", ");
            Serial.print(j);
            Serial.println("] maxVdac");
          }
        }
      }
      EEPROM.write(isensor,iVdac);
      okpoints=0;
      iVdac=0;
    }
  }
  InitVariables();
}

/***************************************************************************** 
  Function:  chars2Int
******************************************************************************/      
int chars2Int(char digit0, char digit1, char digit2){
  
  if (digit0 > 57 || digit0 < 48){
    return 0;
  }
  int value = digit0 - 48; 
  if (digit1 <= 57 || digit1 > 48){
      value += (digit1 - 48)*10;
  }
  if (digit2 <= 57 || digit2 > 48){
      value += (digit2 - 48)*100;
  }
  return value;
}
       
      
    




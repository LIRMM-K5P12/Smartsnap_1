/* Code to save the sensor data into the SD card
    The objective is to have enough measurements to allow an identification of a fish
    To do that, the writing must be the quicker possible in order to make the MCU sleeps between to measure and save energy
*/

////////////////////////////////////////////////////////////////////////
// TO DEFINE FOR EACH DEVICE
////////////////////////////////////////////////////////////////////////
// Device number
String device_number = "010";

// Hard Offsets list need to be up to date, ref Google Sheet
float hard_offsets[][3] = {{-38.94,-34.01,-72.58},{-25.88,-18.33,-72.68},{-15.39,41.85,76.80},{-8.44,-19.42,-63.38},{15.67,-32.52,-91.45},{-7.82,-23.97,-37.19},{11.96,-34.55,-95},{-2.74,-22.77,-62.61},{-3.61,-2.66,-47.82},{11.74,-31.19,-90.35},{-15.45,-41.96,-86},{-23.67,-43.26,-67.86},{-21.85,-43.41,-80.90},{-0.27,-21.66,-45.45},{-10.5,-29.76,-70.55},{-25,-30.82,-75.03},{-36.53,-57.33,-60.94},{-29.93,-23.25,-72.53},{-11.75,-18.49,-72.99},{-7.59,-19.88,-37.70},{-15.41,-41.11,-58.31},{-39.28,-16.63,-68.33},{-17.95,-50.01,-48.22},{-29.5,-44.19,-72.95},{-18.21,-35.76,-66.12},{-24.69,-11.84,-82.67},{-14.02,-37.86,-79.91},{-24.89,-29.42,-64.73},{-26.67,-23.4,-69.09},{-20.78,-57.34,-70.08},{-5.72,-13.91,-44.32},{-13.71,-26.75,-52.62},{-34.88,-54.32,-72.13},{-7.73,-12.79,-64.02},{-20.72,-23.11,-78.30},{-20.02,-42.10,-69.98},{-16.20,-28.42,-60.14},{-20,-31.24,-71.51},{2.53,-13.91,-56.53},{-4.59,-11.22,-70.05},{-37.66,-38.88,-57.04},{-29.08,-22.48,-81.05},{-21.79,-33.55,-68.60},{-33.07,-19.67,-85.46},{-9.10,-33.09,-60.73},{-15.21,-20.71,-78.14},{-31.68,-32.99,-59.69},{-6.32,-20.13,-47.65},{-24.79,-29.62,-60.36},{-28.65,-38.97,-74.80},{21.99,-29.15,-93.15},{8.39,-15.95,-44.12},{6.58,-5.31,-37.97},{11.90,-15.95,-61.08},{44632,-30.09,-91.93},{15.16,-13.52,-82.47},{-31.13,-44.51,-67.97},{8.82,-17.16,-51.51},{-6.67,-14.84,-63.14},{17.30,-2.50,-36.58},{-32.03,-33.64,-60.06},{-7.14,-0.92,-39.71},{-8.66,-27.27,-50.86},{-3.09,-16.81,-62.83},{-32.25,-38.58,-62.86},{-21.04,-17.82,-67.17},{-5.99,-27.32,-51.29},{-4.81,-2.06,-55.65},{-28.31,-32.50,-66.44},{0.28,-12.47,-47.97},{-7.55,-23.07,-63.10},{-0.01,-1.46,-51.81},{-4.05,-13.55,-34.88},{6.14,-18.16,-50.42},{-28.88,-43.36,-73.52},{-5.51,-17.25,-41.44},{-6.29,-29.72,-49.53},{-10.05,-7.35,-49.54},{-18.55,-39.24,-75.17},{18,-23.87,-80.28},{-3.33,-15.41,-45.70},{-5.61,-17.91,-79.82},{-19.48,-36.73,-62.13},{-12.16,-12.97,-66.89},{-27.95,-24.52,-73.10},{-27.30,-35.56,-57.05},{-2.32,-3.18,-47.32},{-37.07,-49.22,-69.08},{-40.06,-42.22,-67.48},{44788,-26.89,-94.88},{-10.95,-12.29,-52.23},{-32.04,-44.75,-57.36},{44613,-35.22,-88.02},{1.26,0.26,-45.65},{-35.35,-53.53,-68.55},{12.51,-39.35,-81.51},{-11.79,-26.20,-50.80},{-16.27,-46.26,-78.88},{-20.80,-42.63,-76.22},{9.16,-26.48,-38.09},{-36.84,-29.63,-69.04},{-46.04,-70.49,-62.83},{-14.11,-31.27,-82.81},{-21.69,-18.80,-82.77},{-6.6,-10.7,-41.38},{-21.40,-21.40,-74.36},{-46.12,-24.58,-64.98},{-15.48,-11.05,-69.13},{-18.2,-37.84,-179.4},{-1.17,-12.74,-37.72},{8.92,-27.77,-42.65},{-27.40,-32.75,-66.96},{4.43,-3.26,-50.58},{-30.42,-30.36,-70.82},{-28.51,-38.16,-66.12},{-11.10,-17.04,-69.18},{2.48,-1.65,-52.99},{-30.86,-16.12,-78.12},{-5.8,-10.28,-59},{-25.95,-49.12,-77.88},{44907,-40.49,-90.50},{-8.3,-32.65,-56.05},{9.75,-16.95,-35.80},{6.28,-21.66,-94.90},{-8.15,-31.52,-75.32},{-19.27,-12.71,-65.36},{-26.72,-41.43,-79.55},{-27.87,-23.52,-56.29},{13.92,-36.78,-80.00},{-6.06,-15.78,-55.98},{-18.18,-54.59,-78.26},{-38.76,-42.63,-60.50},{-24.02,-10.29,-85.44},{11.15,-18.93,-51.92},{-32.80,-44.52,-67.38},{-44.01,-53.48,-59.96},{-27.48,-30.45,-67.26},{-27.20,-30.18,-65.90},{-37.44,-46.07,-71.08},{-15.35,-20.75,-69.44},{-32.46,-42.42,-68.12},{-22.98,-43.24,-84.40},{-37.62,-30.69,-55.60},{5.53,-5.46,-40.87},{-4.2,-31.55,-52.54},{-34.68,-42.69,-70.05},{-24.16,-34.24,-69.18},{-24.33,-42.30,-59.15},{-16.80,-35.97,-85.29},{-39.22,-48.24,-64.44},{-38.06,-36.81,-63.39},{-46.03,-37.55,-71.66},{3.26,-5.68,-56.03},{-30.68,-48.81,-56.61},{-21.12,-23.97,-74.86},{-32.82,-23.26,-67.17},{-40.65,-48.67,-63.08},{-22.40,-47.94,-65.10},{-16.72,-31.52,-63.43},{-22.93,-34.14,-70.02},{-20.99,-19.60,-88.03},{-12.80,-27.99,-65.12},{0.14,-9.13,-60.64},{-18.24,-28.42,-22.94},{-10.50,-10.79,-45.62},{-6.85,-9.77,-47.00},{-18.75,-23.18,-88.62},{-18.63,-14.75,-84.43},{-21.37,-35.30,-79.70},{-25.98,-41.53,-74.09},{-22.77,-38.62,-159.10},{12.72,-36.77,-97.68},{-13.22,-17.28,-45.01},{7.95,-21.71,-49.93},{-13.42,-23.21,-60.60},{-51.15,-57.75,-50.38},{-35.32,-40-70,-57.57},{0.77,0.21,-64.44},{-23.09,-19.47,-70.49},{-26.35,-33.48,-65.28},{-18.94,-32.06,-73.02},{2.86,-0.49,-51.26},{-26.42,-34.25,-69.39},{-17.43,-12.12,-73.08},{-6.45,-25.20,-65.22},{-29.79,-42.41,-63.94},{-9.88,-15.11,-56.62},{-23.57,-45.41,-163.69},{12.32,-10.45,-55.36},{-17.14,-7.26,-79.01},{10.36,-22.85,-58.79},{15.92,-17.09,-91.16},{-10.14,-25.78,-57.74},{-15.04,-33.13,-170.84},{-23.15,-46.08,-73.56},{6.82,-4.89,-46.33},{-14.71,-36.48,-166.03},{-14.97,-49.44,-64.44},{-17.37,-16.32,-38.25},{21.89,-21.05,-76.40},{-39.21,-35.20,-49.84},{-34.20,-47.94,-66.45},{-10.15,-15.42,-45.50},{-40.06,-49.92,-67.21},{-20.12,-5.79,-67.02},{-2.30,-5.99,-45.43},{-26.82,-39.66,-79.26},{-43.94,-36.09,-60.72},{-23.92,-29.63,-70.79},{-2.24,-6.52,-55.86},{-13.55,-18.00,-63.65},{-3.8,-13.88,-60.18},{-4.36,-9.74,-44.76},{-8.10,-23.91,-73.29},{-20.15,-20.72,-67.44},{4.68,-3.68,-49.19},{-13.27,-42.95,-72},{-14.51,-33.27,-73.86}};

////////////////////////////////////////////////////////////////////////
// INCLUDE
////////////////////////////////////////////////////////////////////////
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ArduinoLowPower.h>
#include <MS5837.h>
#include <TimeLib.h>
#include <RTCZero.h>
////////////////////////////////////////////////////////////////////////
// Define
////////////////////////////////////////////////////////////////////////
#define VBAT_PIN A7
#define VUSB_PIN A1

////////////////////////////////////////////////////////////////////////
// RTC Zero Initialization
////////////////////////////////////////////////////////////////////////
/* Create an rtc object */
RTCZero rtc;

////////////////////////////////////////////////////////////////////////
// Low Power configuration
////////////////////////////////////////////////////////////////////////
// Flag true if awake, false if sleep
bool pressure_flag = false;
bool usb_flag = false;

////////////////////////////////////////////////////////////////////////
// USB cable detection
////////////////////////////////////////////////////////////////////////
uint16_t usb_threshold = 950;
bool serial_init_bool = false;

////////////////////////////////////////////////////////////////////////
// MS5837 configuration
////////////////////////////////////////////////////////////////////////
// MS5837 definition
MS5837 pressure_temp;

// Threshold
uint16_t pressure_threshold = 1300;

// Time between to measure
uint16_t nbr_loop = 45;
uint16_t actual_loop = nbr_loop - 1;

// Buffer to measure every second
float buffer_pression, buffer_temperature;

////////////////////////////////////////////////////////////////////////
// NXP 9DOF configuration
////////////////////////////////////////////////////////////////////////
// Gyroscope definition
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

// Accelerometer and magnetometer definition
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

////////////////////////////////////////////////////////////////////////
// Frequency configuration
////////////////////////////////////////////////////////////////////////
// Sampling frequency
#define SAMPLING_FREQ 100 // en Hz
uint32_t sampling_delay = 1000 / SAMPLING_FREQ;
// int sampling_delay = 0;

////////////////////////////////////////////////////////////////////////
// SD Card configuration
////////////////////////////////////////////////////////////////////////

// On the Ethernet Shield, CS is pin 4. SdFat handles setting SS
const int chipSelect = 4;

// SD card definition
SdFat SD;
File myFile;
File root;

// Data is stored according to this structure
// Important : this structure must be reported in : "convert raw data to csv.py"
struct datastore
{
  float acc_x;
  float acc_y;
  float acc_z;
  float gyr_x;
  float gyr_y;
  float gyr_z;
  float mag_x;
  float mag_y;
  float mag_z;
  float temp;
  float pressure;
  uint32_t time;
};
datastore myData;

int i_loop = 0;
int i = 0;

boolean open_file = false;
boolean recording = false;
uint8_t recording_number = 0;
uint8_t deployment_number = 0;

float battery_threshold = 3.25;
String monitoring_file_name = "test.dat";

String date_of_upload = __DATE__;
String time_of_upload = __TIME__;

void setup()
{
  // Define pin to light the LED
  pinMode(VUSB_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  ////////////////////////////////////////////////////////////////////////
  // Serial Init
  ////////////////////////////////////////////////////////////////////////
  Serial.begin(115200);

  // delay in order to establish the connection
  delay(5000);

  // while(!Serial){
  //   delay(1);
  // }

  // prints time since program started
  Serial.println("Setup");
  Serial.println(millis());

  ////////////////////////////////////////////////////////////////////////
  // SD Card initialisation
  ////////////////////////////////////////////////////////////////////////
  // init SD card
  Serial.print("Initializing SD card...");

  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library.
  // change to SPI_FULL_SPEED for more performance.
  SDCardCheck();

  // end init SD card
  Serial.println("initialization done.");

  root = SD.open("/");
  deployment_number = whereToStart(root,0);
  Serial.print("La carte SD contient déjà ");
  Serial.print(deployment_number);
  Serial.println(" fichiers de données");

  // createFileSDCard("Battery_state.txt");
  myFile.open("Battery_state.txt", O_RDWR | O_CREAT | O_AT_END);
  myFile.print(date_of_upload);
  myFile.print(" - ");
  myFile.println(time_of_upload);
  myFile.close();

  ////////////////////////////////////////////////////////////////////////
  // I2C communication initialisation
  ////////////////////////////////////////////////////////////////////////
  // Open I2C communication
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  ////////////////////////////////////////////////////////////////////////
  // Sensors initialisation
  ////////////////////////////////////////////////////////////////////////
  sensors_init();
  pressure_temp.read();
  Serial.print("Here  we go !");
  Serial.end();

  ////////////////////////////////////////////////////////////////////////
  // RTC Zero Initialization
  ////////////////////////////////////////////////////////////////////////
  rtc.begin();
  rtc.setTime(0, 0, 0); 

  ////////////////////////////////////////////////////////////////////////
  // Low Power Initialization
  ////////////////////////////////////////////////////////////////////////
  // LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, alarmMatch, CHANGE);
}

void loop()
{
  // serialPrint_myData();
  // delay(500);
  // /*
  // USB cable detection
  // Close SD file and open serial communication
  if (usb_flag == true){
    digitalWrite(LED_BUILTIN,HIGH);
    myFile.close();
    ihm();
    digitalWrite(LED_BUILTIN,LOW);
    open_file = true;
  }

  // Pressure detection
  // Saving start of monitoring 
  // Opening/Creating SD file
  // Saving data
  if(pressure_flag == true){
    // Go through this condition once when start monitoring
    // digitalWrite(LED_BUILTIN,HIGH);
    if(open_file==true){
      // Save the start of monitoring
      recording_number++;
      saveRecordings(true);
      recording=true;
      // Check if the SD is accessible
      SDCardCheck();
      // Sensors initialisation before stating monitoring
      sensors_init();
      // Saving battery state before starting monitoring
      saveBatteryState();
      // Create and open the file for monitoring
      monitoring_file_name = "S_Device_"+String(device_number)+"_data_"+String(deployment_number)+".dat";
      myFile = SD.open(monitoring_file_name, O_RDWR | O_CREAT | O_AT_END);
      // Increment the number of deployment
      deployment_number++;
      // Close the condition
      open_file = false;
    }
    Sensors_sd_write();
  }

  // Saving end of monitoring
  // Go to sleep mode
  if((analogRead(VUSB_PIN) < usb_threshold)&&(pressure_temp.pressure() < pressure_threshold)){
    int x = 1;
    // Go through this condition once when stop monitoring
    if(recording==true){
      // Save the end of the monitoring
      saveRecordings(false);
      // Sensors initialisation before stating monitoring
      sensors_init();
      // Saving battery state before starting monitoring
      saveBatteryState();
      recording=false;
      // Set x=10 for a 10min sleep
      x = 10;
    }
    // Close the file before going to sleep
    myFile.close();

    // Put all the flags down
    pressure_flag = false;
    usb_flag = false;
    open_file = true;
    
    // Set a Xmin sleep
    setAlarm_Xmin(x);
    // Initialization of pin, SC card and sensors
    init_all();
    // Reading the pressure
    pressure_temp.read();
  }
  // */
}

void init_all(){
  // Define USB pin
  pinMode(VUSB_PIN, INPUT);
  // Sensors initialisation in order to read the pressure
  sensors_init();
  // SD Card init
  SDCardCheck();
}

void displaySensorDetails(void) {
  sensor_t accel, mag;
  accelmag.getSensor(&accel, &mag);
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(accel.name);
  Serial.print("Driver Ver:   ");
  Serial.println(accel.version);
  Serial.print("Unique ID:    0x");
  Serial.println(accel.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(accel.max_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(accel.min_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(accel.resolution, 8);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(mag.name);
  Serial.print("Driver Ver:   ");
  Serial.println(mag.version);
  Serial.print("Unique ID:    0x");
  Serial.println(mag.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(mag.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(mag.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(mag.resolution);
  Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void add0digit(int y){
  if(y<10) Serial.print("0");
}

void blinkLed(int time){
  digitalWrite(LED_BUILTIN,HIGH);
  delay(time);
  digitalWrite(LED_BUILTIN,LOW);
  delay(time);
}

void SDCardCheck(){
  if (!SD.begin(chipSelect, SPI_HALF_SPEED)){
    digitalWrite(LED_BUILTIN,HIGH);
    myFile.open("error.txt", O_RDWR | O_CREAT | O_AT_END);
    myFile.println("Carte SD failed");
    myFile.close();
    SD.initErrorHalt();
  }
}

int whereToStart(File dir, int numTabs) {
    int position = 0;
    int max_characters = 25;
    char f_name[max_characters];
    String entry_name;
    position++;
    while (true) {
        File entry =  dir.openNextFile();

        if (! entry) {
            // no more files
            break;
        }

        for (uint8_t i = 0; i < numTabs; i++) {
            Serial.print('\t');
        }

        entry.getName(f_name, max_characters);
        entry_name=String(f_name);

        if (entry.isDirectory()){}
        else{
            if(entry_name.substring(0,6)=="Device"){
                position++;
            }
        }
        entry.close();
    }
    return position;
}


void sensors_init(){
  bool sensor_error = false;
  myFile.open("Sensors_init.txt", O_RDWR | O_CREAT | O_AT_END);
  myFile.print("Sensors initialisation at ");
  myFile.println(millis());
  ////////////////////////////////////////////////////////////////////////
  // MS5837 initialisation
  ////////////////////////////////////////////////////////////////////////
  if (!pressure_temp.init()) {
    myFile.println("Init failed!");
    myFile.println("Are SDA/SCL connected correctly?");
    myFile.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    myFile.println("\n\n\n");
    delay(5000);
    sensor_error = true;
  }

  pressure_temp.setModel(MS5837::MS5837_30BA);
  pressure_temp.setFluidDensity(1030); // kg/m^3 (freshwater, 1029 for seawater)

  ////////////////////////////////////////////////////////////////////////
  // NXP 9DOF initialisation
  //////////////////////////////////////////////////////////////////////
  // Initialise the gyroscope
  if (!gyro.begin()) {
    // There was a problem detecting the FXAS21002C ... check your connections
    myFile.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    sensor_error = true;
  }

  // Initialise the accelerometer and magnetometer
  if (!accelmag.begin()) {
    // There was a problem detecting the FXOS8700 ... check your connections
    myFile.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    sensor_error = true;
  }
  
  if(sensor_error == false) myFile.println("All sensors initialised !");
  myFile.close();
  if(sensor_error == true){
    while(1){
      blinkLed(5000);
    }
  }
}

void printDirectory(File dir, int numTabs) {
  int position = 0;
  int max_characters = 25;
  char f_name[max_characters];
  String entry_name;
  Serial.print(position);
  Serial.println(" - Quitter le menu");
  position++;
  while (true) {
    File entry =  dir.openNextFile();
    
    if (! entry) {
      // no more files
      break;
    }

    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    entry.getName(f_name, max_characters);
    entry_name=String(f_name);
    
    if (entry.isDirectory()){}
    else{
    // if(entry_name!="System Volume Information"){
      Serial.print(position);
      Serial.print(" - ");
      Serial.print(entry_name);
      Serial.print("\t");
      Serial.print(entry.size(), DEC);
      Serial.println(" octets");
      position++;
    }
    entry.close();
  }
  Serial.println("998 - Supprimer un fichier");
  Serial.println("999 - Supprimer tous les fichiers de la carte SD");
  Serial.println("Tapez le numéro de la ligne pour accéder au fichier");
  Serial.println("");
}

void readSDfile(File dir, int file_number){
  File fileToRead;
  int max_characters = 25;
  char f_name[max_characters];
  int i=1;
  while (true) {
    // Serial.println(i);
    File entry =  dir.openNextFile();
    if (! entry) {
      Serial.println("Erreur - Mauvais numéro");
      // no more files
      break;
    }
    if (entry.isDirectory()){}
    else{
      if(file_number == i){
        if(entry.size()==0){
          Serial.println("Fichier vide");
        }else{
          entry.getName(f_name, max_characters);
          String file_name = String(f_name);
          // Serial.println(file_name);
          // Serial.println(file_name.substring(file_name.length()-4,file_name.length()));
          fileToRead = SD.open(f_name, O_READ);
          if(file_name.substring(file_name.length()-4,file_name.length())==".txt"){
            while (fileToRead.available()) {
              Serial.write(fileToRead.read());
            }
          }
          if(file_name.substring(file_name.length()-4,file_name.length())==".dat"){
            Serial.print("Starts at ");
            Serial.println(millis());
            while (fileToRead.available()) {
              struct datastore readData;
              fileToRead.read((uint16_t *)&readData, sizeof(readData));
              serialPrintData(readData);
            }
            Serial.print("Ends at ");
            Serial.println(millis());
          }
          fileToRead.close();
        }
        break;
      }
      i++;
    }
  }
}

void serialPrintData(datastore writtenData){
    String dataString = "";
    dataString += writtenData.acc_x;
    dataString += ",";
    dataString += writtenData.acc_y;
    dataString += ",";
    dataString += writtenData.acc_z;
    dataString += ",";
    dataString += writtenData.gyr_x;
    dataString += ",";
    dataString += writtenData.gyr_y;
    dataString += ",";
    dataString += writtenData.gyr_z;
    dataString += ",";
    dataString += writtenData.mag_x;
    dataString += ",";
    dataString += writtenData.mag_y;
    dataString += ",";
    dataString += writtenData.mag_z;
    dataString += ",";
    dataString += writtenData.temp;
    dataString += ",";
    dataString += writtenData.pressure;
    dataString += ",";
    dataString += writtenData.time;
    Serial.println(dataString);
    // Serial.println("");
}

void ihm(){
  int user_input = 1;
  Serial.begin(115200);
  SDCardCheck();
  while(!Serial){
    delay(1);
    // if(analogRead(VUSB_PIN) < usb_threshold) break;
  }

  while((user_input!=0)||(analogRead(VUSB_PIN) < usb_threshold)){
    Serial.println("");
    Serial.println("Voici l'ensemble des fichiers sur la carte SD :");
    root = SD.open("/");
    printDirectory(root, 0);
    
    while(Serial.available()<1){
      delay(1);
      // if(analogRead(VUSB_PIN) < usb_threshold) break;
    }
    // if (analogRead(VUSB_PIN) < usb_threshold) break;
    user_input = Serial.readString().toInt();
    Serial.print("Entrée utilisateur : ");
    Serial.println(user_input);
    switch(user_input){
      case 999:
        removeAllFiles(root);
        break;
      case 998:
        removeOneFile(root);
        break;
      case 0:
        break;
      default:
        readSDfile(root,user_input);
        break;
    }   
  }

  Serial.println("Débranchez l'USB");
  Serial.end();
  
  while(analogRead(VUSB_PIN) > usb_threshold-1){
    delay(1);
  }
}

void removeAllFiles(File dir){
  while(true){
    Serial.println("Etes-vous sûr(e) ? Oui / Non");
    while(Serial.available()<1){
      delay(1);
    }
    String answer = Serial.readString();
    if(answer =="Oui"){
      Serial.println("Suppression des fichiers en cours");
      int position = 0;
      int max_characters = 25;
      char f_name[max_characters];
      String entry_name;
      position++;
      while (true) {
        File entry =  dir.openNextFile();
        
        if (! entry) {
          // no more files
          break;
        }

        entry.getName(f_name, max_characters);
        entry_name=String(f_name);
        
        if (entry.isDirectory()){}
        else{
        // if(entry_name!="System Volume Information"){
          SD.remove(entry_name);
          position++;
        }
        entry.close();
      }
      Serial.println("Suppression des fichiers terminée");
      break;
    }else if(answer =="Non"){
      Serial.println("Retour au menu");
      break;
    }else{
      Serial.println("Réponse non reconnue (Attention à bien mettre les majuscules)");
    }
  }
}

void removeOneFile(File dir){
  Serial.println("Rentrez le numéro de ligne correspondant au fichier à supprimer :");
  while(Serial.available()<1){
    delay(1);
  }
  int file_number = Serial.readString().toInt();
  Serial.println("Suppression des fichiers en cours");
  int position = 0;
  int max_characters = 25;
  char f_name[max_characters];
  String entry_name;
  position++;
  while (true) {
    File entry =  dir.openNextFile();
    
    if (! entry) {
      Serial.println("Aucun fichier supprimé");
      // no more files
      break;
    }

    entry.getName(f_name, max_characters);
    entry_name=String(f_name);
    
    if (entry.isDirectory()){}
    else{
      if(file_number==position){
        SD.remove(entry_name);
        Serial.print("Suppression de ");
        Serial.print(entry_name);
        Serial.println(" faite");
        break;
      }
      position++;
    }
    entry.close();
  }
}

void saveRecordings(boolean start){
  // String line = "Recording number "+recording_number+" - starts at "+millis();
  myFile.open("Recording_list.txt", O_RDWR | O_CREAT | O_AT_END);
  myFile.print("Recording number ");
  myFile.print(recording_number);
  if(start==true){
    myFile.print(" - starts at ");
  }else{
    myFile.print(" - ends at ");
  }
  myFile.println(millis());
  myFile.close();  
}

void saveBatteryState(){
  float battery_level = analogRead(VBAT_PIN)*2*3.3/1024;
  myFile.open("Battery_state.txt", O_RDWR | O_CREAT | O_AT_END);
  myFile.print(battery_level);
  myFile.print(" - millis : ");
  myFile.print(millis());
  myFile.print(" - minutes : ");
  myFile.println(rtc.getMinutes()); 
  myFile.close();
  if(battery_level<battery_threshold) batteryAlert();
}

void batteryAlert(){
  float battery_level = analogRead(VBAT_PIN)*2*3.3/1024;
  while(battery_level<battery_threshold+0.5){
    blinkLed(500);
    battery_level = analogRead(VBAT_PIN)*2*3.3/1024;
  }
}

void saveTest(int value){
  pressure_temp.read();
  myFile.open("Test.txt", O_RDWR | O_CREAT | O_AT_END);
  myFile.print(pressure_temp.pressure());
  myFile.print(" - ");
  myFile.println(value);
  // myFile.println(analogRead(VUSB_PIN));
  myFile.close();
}

// Create a SD file
void createFileSDCard(String name){
  myFile = SD.open(name, O_RDWR | O_CREAT | O_AT_END);
  myFile.close();
}

// Wake up the microcontroller in X min
void setAlarm_Xmin(int x){
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  uint32_t milli_sec_value = x*60*1000;
  USBDevice.detach();
  SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk;
  LowPower.idle(milli_sec_value); 
  SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
  USBDevice.attach(); 
  alarmMatch();
}

// Read the pressure and if it's more than X bars, wake up the microcontroller
void alarmMatch(){
  // Initialization of pin, SC card and sensors
  init_all();
  // Reading pressure
  pressure_temp.read();
  if (pressure_temp.pressure() > pressure_threshold-1)
  {
    pressure_flag = true;
  }
  if (analogRead(VUSB_PIN) > usb_threshold-1)
  {
    usb_flag = true;
  }
  if((rtc.getMinutes()==1)||(rtc.getMinutes()==31)){
    saveBatteryState();
  }
}

// Serial port initialisation
void serial_init()
{
  // Serial port speed setting
  Serial.begin(115200);

  // Wait for the Serial Monitor
  // while (!Serial) {
  //   delay(1);
  // }
  delay(10000);
}

// Printing the data to the serial port.
void serialPrint_myData()
{
  // Serial.begin(115200);
  // delay(5000);
  read_all_sensors();

  Serial.print("gyr x : ");
  Serial.println(myData.gyr_x);
  Serial.print("gyr y :");
  Serial.println(myData.gyr_y);
  Serial.print("gyr z :");
  Serial.println(myData.gyr_z);
  Serial.println("");

  Serial.print("acc x : ");
  Serial.println(myData.acc_x);
  Serial.print("acc y :");
  Serial.println(myData.acc_y);
  Serial.print("acc z :");
  Serial.println(myData.acc_z);
  Serial.println("");

  Serial.print("Mag x : ");
  Serial.println(myData.mag_x);
  Serial.print("Mag y :");
  Serial.println(myData.mag_y);
  Serial.print("Mag z :");
  Serial.println(myData.mag_z);
  Serial.println("");

  Serial.print("Pression :");
  Serial.println(myData.pressure);
  Serial.println("");

  Serial.print("Temperature :");
  Serial.println(myData.temp);
  Serial.println("");

  Serial.print("Temps :");
  Serial.println(myData.time);
  Serial.println("");
  // Serial.end();
}

// Read all the sensors and store the data in the structure
void read_all_sensors()
{
  actual_loop++;
  // Get a new sensor event
  sensors_event_t gevent, aevent, mevent;
  gyro.getEvent(&gevent);
  accelmag.getEvent(&aevent, &mevent);

  pressure_temp.read();
  buffer_pression = pressure_temp.pressure();
  buffer_temperature = pressure_temp.temperature();
  actual_loop = 0;

  // uint32_t boucle = 0;
  int16_t ax, ay, az, t, gx, gy, gz;

  // while(boucle<1200000){
  // Read data from the MPU6050
  // readRawData(MPU_ADDRESS_1, ax, ay, az, t, gx, gy, gz);

  // dispatch the data into the struct
  myData.time = millis();
  myData.pressure = buffer_pression;
  myData.temp = buffer_temperature;     //((float)t / 340.0) + 36.53;
  myData.acc_x = aevent.acceleration.x; //((float)ax) * accel_calibration;
  myData.acc_y = aevent.acceleration.y; //((float)ay) * accel_calibration;
  myData.acc_z = aevent.acceleration.z; //((float)az) * accel_calibration;
  myData.gyr_x = gevent.gyro.x;         //((float)gx) * gyro_calibration;
  myData.gyr_y = gevent.gyro.y;         //((float)gy) * gyro_calibration;
  myData.gyr_z = gevent.gyro.z;         //((float)gz) * gyro_calibration;
  myData.mag_x = mevent.magnetic.x-hard_offsets[device_number.toInt()-1][0];     //((float)ax) * accel_calibration;
  myData.mag_y = mevent.magnetic.y-hard_offsets[device_number.toInt()-1][1];     //((float)ay) * accel_calibration;
  myData.mag_z = mevent.magnetic.z-hard_offsets[device_number.toInt()-1][2];     //((float)az) * accel_calibration;
}

// Write a certain amount of data into the SD card
void Sensors_sd_write()
{  
  // Get a new sensor event
  sensors_event_t gevent, aevent, mevent;
  Serial.print("gyro starts at : ");
  Serial.println(millis());
  gyro.getEvent(&gevent);
  // Serial.print("gyro ends at : ");
  // Serial.println(millis());
  // Serial.println("");

  // Serial.print("accelmag starts at : ");
  // Serial.println(millis());
  accelmag.getEvent(&aevent, &mevent);
  // Serial.print("accelmag ends at : ");
  // Serial.println(millis());
  // Serial.println("");
  // uint32_t boucle = 0;
  int16_t ax, ay, az, t, gx, gy, gz;  
  
  actual_loop++;
  if (actual_loop == nbr_loop)
  {
    pressure_temp.read();
    if(pressure_temp.pressure()<0){
      myFile.close();
      myFile.open("Sensors_Reinit.txt", O_RDWR | O_CREAT | O_AT_END);
      myFile.print("Réinitialisation capteur - ");
      myFile.println(millis());
      myFile.close();
      init_all();
      pressure_temp.read();
      myFile = SD.open(monitoring_file_name, O_RDWR | O_CREAT | O_AT_END);
    }
    buffer_pression = pressure_temp.pressure();
    buffer_temperature = pressure_temp.temperature();
    actual_loop = 0;
    myFile.sync();
  }else{
    delay(7);
  }

  // Serial.print("sd open starts at : ");
  // Serial.println(millis());
  // //myFile = SD.open(name, O_RDWR | O_CREAT | O_AT_END);
  // Serial.print("sd open ends at : ");
  // Serial.println(millis());
  // Serial.println("");

  // Serial.print("data starts at : ");
  // Serial.println(millis());
  // dispatch the data into the struct
  myData.time = millis();
  myData.pressure = buffer_pression;
  myData.temp = buffer_temperature;     //((float)t / 340.0) + 36.53;
  myData.acc_x = aevent.acceleration.x; //((float)ax) * accel_calibration;
  myData.acc_y = aevent.acceleration.y; //((float)ay) * accel_calibration;
  myData.acc_z = aevent.acceleration.z; //((float)az) * accel_calibration;
  myData.gyr_x = gevent.gyro.x;         //((float)gx) * gyro_calibration;
  myData.gyr_y = gevent.gyro.y;         //((float)gy) * gyro_calibration;
  myData.gyr_z = gevent.gyro.z;         //((float)gz) * gyro_calibration;
  myData.mag_x = mevent.magnetic.x-hard_offsets[device_number.toInt()-1][0];     //((float)ax) * accel_calibration;
  myData.mag_y = mevent.magnetic.y-hard_offsets[device_number.toInt()-1][1];     //((float)ay) * accel_calibration;
  myData.mag_z = mevent.magnetic.z-hard_offsets[device_number.toInt()-1][2];    //((float)az) * accel_calibration;
  // Serial.print("data ends at : ");
  // Serial.println(millis());
  // Serial.println("");

  // write the data into the SD card
  // Serial.print("sd write starts at : ");
  // Serial.println(millis());
  myFile.write((const uint8_t *)&myData, sizeof(myData));
  // Serial.print("sd write ends at : ");
  // Serial.println(millis());
  // Serial.println("");

  // Serial.print("sd sync starts at : ");
  // Serial.println(millis());
  // myFile.flush();
  // Serial.print("sd sync ends at : ");
  // Serial.println(millis());
  // Serial.println("");
}
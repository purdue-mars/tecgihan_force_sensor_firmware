#include "SdFat.h"

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 0
/*
  Change the value of SD_CS_PIN if you are using SPI and
  your hardware does not use the default value, SS.  
  Common values are:
  Arduino Ethernet shield: pin 4
  Sparkfun SD shield: pin 8
  Adafruit SD shields and modules: pin 10
*/

/*// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
#endif  // HAS_SDIO_CLASS*/

const int8_t DISABLE_CS_PIN = -1;

const uint8_t SD_CS_PIN = 10;

#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50))

// Try to select the best SD_FAT_TYPE.
#if SD_FAT_TYPE == 0
SdFat sd;
File file;
File root;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
File32 root;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
ExFile root;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
FsFile root;
#endif  // SD_FAT_TYPE

char line[100];
char QDevInfoStr[500];
uint32_t SrNo;

//------------------------------------------------------------------------------
// Store error strings in flash to save RAM.
#define error(s) sd.errorHalt(&Serial, F(s))
//------------------------------------------------------------------------------
// Check for extra characters in field or find minus sign.
char* skipSpace(char* str) {
  while (isspace(*str)) str++;
  return str;
}
//------------------------------------------------------------------------------
bool parseLine(char* str) {
  char* ptr;

  // Set strtok start of line.
  str = strtok(str, ",");
  if (!str) return false;
  
  // Print text field.
  Serial.println(str);
  
  // Subsequent calls to strtok expects a null pointer.
  str = strtok(nullptr, ",");
  if (!str) return false;
  
  // Convert string to long integer.
  int32_t i32 = strtol(str, &ptr, 0);
  if (str == ptr || *skipSpace(ptr)) return false;
  Serial.println(i32);
  
  str = strtok(nullptr, ",");
  if (!str) return false;
  
  // strtoul accepts a leading minus with unexpected results.
  if (*skipSpace(str) == '-') return false;
  
  // Convert string to unsigned long integer.
  uint32_t u32 = strtoul(str, &ptr, 0);
  if (str == ptr || *skipSpace(ptr)) return false;
  Serial.println(u32);

  str = strtok(nullptr, ",");
  if (!str) return false;
  
  // Convert string to double.
  double d = strtod(str, &ptr);
  if (str == ptr || *skipSpace(ptr)) return false;
  Serial.println(d);
  
  // Check for extra fields.
  return strtok(nullptr, ",") == nullptr;
}
//------------------------------------------------------------------------------
cid_t m_cid;

//static ArduinoOutStream cout(Serial);
//----------------------------CARD INFO-----------------------------------------
bool cidDmp() {
 // static uint32_t SdInfo[3];
  Serial.println("\nSD Card INFO");
  Serial.print(F("Manufacturer ID: "));
  Serial.println((int)(m_cid.mid),HEX);
  Serial.print(F("OEM ID: ")); Serial.print(m_cid.oid[0]);
  Serial.println(m_cid.oid[1]);
  Serial.print(F("Product #: "));
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(m_cid.pnm[i]);
  }
  Serial.print(F("\nVersion: "));
  Serial.print((int)(m_cid.prv_n)); Serial.print(F("."));
  Serial.println((int)(m_cid.prv_m));

  Serial.print(F("Serial #: "));
  Serial.println(m_cid.psn,HEX);
  Serial.println(m_cid.psn,DEC);
  Serial.println();
  Serial.println(m_cid.psn); 
  //SdInfo[0] = m_cid.psn;

  ///Serial.println(SdInfo[0]);

  Serial.print(F("Manufacturing Date: "));
  Serial.print(m_cid.mdt_month); Serial.print(F("/"));
  Serial.println(2000 + m_cid.mdt_year_low + 10 * m_cid.mdt_year_high);
  Serial.println();
  return true;
  /*cout << uppercase << showbase << hex << int(m_cid.mid) << dec << endl;
  cout << F("OEM ID: ") << m_cid.oid[0] << m_cid.oid[1] << endl;
  cout << F("Product: ");
  for (uint8_t i = 0; i < 5; i++) {
    cout << m_cid.pnm[i];
  }
  cout << F("\nVersion: ");
  cout << int(m_cid.prv_n) << '.' << int(m_cid.prv_m) << endl;
  cout << F("Serial number: ") << hex << m_cid.psn << dec << endl;
  cout << F("Manufacturing date: ");
  cout << int(m_cid.mdt_month) << '/';
  cout << (2000 + m_cid.mdt_year_low + 10 * m_cid.mdt_year_high) << endl;
  cout << endl;
  return true;*/
}
//------------------------------------------------------------------------------
//-------------------------Main Program-----------------------------------------
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  
  // Wait for USB Serial 
  while (!Serial) {
    yield();
  }
  Serial.println("Type any character to start");
  while (!Serial.available()) {
    yield();
  }
  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
    return;
  }

  //Print Card Info

  if (!sd.card()->readCID(&m_cid)) {
    error("CardReadInfo failed\n");
   // errorPrint();
    return;
  } 
  cidDmp();
  SrNo = m_cid.psn;
  Serial.print(F("Card Serial Num: "));
  Serial.println(SrNo);
  //csdDmp();

  //---------Remove all files related to Device Information-------------
  if (sd.exists("ProductInformation")
    || sd.exists("ProductInformation/DeviceInfo.txt")
    || sd.exists("ProductInformation/HardwareInfo.txt")) {    
    //error("Please remove existing ProductInformation Folder, DeviceInfo.txt and HardwareInfo.txt");
    Serial.println("\nremoving previous files");
    sd.remove("ProductInformation/DeviceInfo.txt");
    sd.remove("ProductInformation/HardwareInfo.txt");
    sd.rmdir("ProductInformation");
    Serial.println("Previous Files removed");
  }
  else{
    Serial.println("ProductInformation Doesn't Exist");
    }

  int rootFileCount = 0;
  if (!root.open("/")) {
    error("open root");
  }
  while (file.openNext(&root, O_RDONLY)) {
    if (!file.isHidden()) {
      rootFileCount++;
    }
    file.close();
    if (rootFileCount > 10) {
      error("Too many files in root. Please use an empty SD.");
    }
  }
  Serial.print("\nRoot file count: ");
  Serial.println(rootFileCount);
  
  if (rootFileCount) {
    Serial.println(F("\nPlease use an empty SD for best results.\n\n"));
    delay(1000);
  }
  // Create a new folder.
  if (!sd.mkdir("ProductInformation")) {
    error("Create ProductInformation failed");
  }
  Serial.println(F("Created ProductInformation Folder\n"));

  // Create a file in Folder1 using a path.
  if (!file.open("ProductInformation/DeviceInfo.txt", O_WRONLY | O_CREAT)) {
    error("create ProductInformation/DeviceInfo.txt failed");
  }
  sprintf(QDevInfoStr,
    "QSTM Device\r\n\n"
    "Product Info\r\n\n"
    "Product Serial No: QL1660320462\r\n"// QL-UnixTime 
    "Product No: QL100N\r\n" // QSTM Localized 100 Newtons  
    "Version: v2K22b\r\n"//v year revision
    "Hardware ID: QL220303\r\n"//QL loadCell Serial 
    //"Serial No: %lu\r\n",SrNo//  %lu formate specifier for unsinged long int
    );      
  file.print(QDevInfoStr);
  file.print(F("Serial No (Hex): "));
  file.print(SrNo,HEX);
  file.print("\n");
  file.close();
  Serial.println(F("Created ProductInformation/DeviceInfo.txt\n"));

  // Change volume working directory to ProductInformation.
  if (!sd.chdir("ProductInformation")) {
    error("chdir failed for folder ProductInformation.\n");
  }
  Serial.println(F("changed working directory to ProductInformation\n"));
  
  // Create HardwareInfo.txt in current directory.
  if (!file.open("HardwareInfo.txt", O_WRONLY | O_CREAT)) {
    error("create HardwareInfo.txt failed");
  }
  //Serial.println("HardwareInfo.txt Created but not written");
  sprintf(QDevInfoStr,
    "Load Cell Info\r\n\n"
    "Serial Number: UL220303\r\n"
    "Product Number: USL06-H12-100N-AP\r\n"// QL-UnixTime
    );

  file.print(QDevInfoStr);
  sprintf(QDevInfoStr,
    "\nCalibration Info\r\n"
    "FCL11 = 25.6015, FCL12 = -1.2078,  FCL13 = -0.7097\r\n"
    "FCL21 =  0.3287, FCL22 = 24.2049, FCL23 = 1.7050\r\n"
    "FCL31 = -0.5876, FCL32 = 0.5815,  FCL33 = 50.1527\r\n"   
    );

  file.print(QDevInfoStr);
  file.close();
  Serial.println(F("Created HardwareInfo.txt in current directory\n"));

  Serial.println(F("\nList of files on the SD.\n"));
  sd.ls("/", LS_R);

  // Change current directory to root.
  if (!sd.chdir()) {
    error("chdir to root failed.\n");
  }  
  // Remove any existing file.
  if (sd.exists("ProductInfo.txt")) {
    sd.remove("ProductInfo.txt"); 
  }
  // Create the file.
  if (!file.open("ProductInfo.txt", FILE_WRITE)) {
    error("open failed");
  }
  // Write test data.
  sprintf(QDevInfoStr,
    "QSTM Device\r\n\n"
    "Product Info\r\n\n"
    "Product Serial No: QL1660320462\r\n"// QL-UnixTime
    "Product No: QL100N\r\n" // QSTM Localized 100 Newtons
    "Version: v2K22b\r\n"//v year revision
    "Hardware ID: QL220303\r\n"//QL loadCell Serial
    //"Serial No: %lu\r\n",SrNo//  %lu formate specifier for unsinged long int
    );
  file.print(QDevInfoStr);
  file.print(F("Serial No (Hex): "));
  file.print(SrNo,HEX);
  file.print("\n");

  sprintf(QDevInfoStr,
    "\nLoad Cell Info\r\n"
    "Serial Number: UL220303\r\n"
    "Product Number: USL06-H12-100N-AP\r\n"// QL-UnixTime
    );

  file.print(QDevInfoStr);

  sprintf(QDevInfoStr,
    "\nCalibration Info\r\n"
    "FCL11 = 25.6015, FCL12 = -1.2078,  FCL13 = -0.7097\r\n"
    "FCL21 =  0.3287, FCL22 = 24.2049, FCL23 = 1.7050\r\n"
    "FCL31 = -0.5876, FCL32 = 0.5815,  FCL33 = 50.1527\r\n"
    );

  file.print(QDevInfoStr);
    
  Serial.println(F("Done Writing"));
  /*  
  // Rewind file for read.
  file.rewind();
  
  while (file.available()) {
    int n = file.fgets(line, sizeof(line));
    if (n <= 0) {
      error("fgets failed"); 
    }
    if (line[n-1] != '\n' && n == (sizeof(line) - 1)) {
      error("line too long");
    }
    if (!parseLine(line)) {
      error("parseLine failed");
    }
    Serial.println();
  }*/
  file.close();

  // Finally list the sd card files

  Serial.println(F("\nList of files on the SD.\n"));
  sd.ls("/", LS_R);
  
  Serial.println(F("Done"));
}

void loop() {
}

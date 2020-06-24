#include <pms3003.h>

#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>


unsigned int pm1 = 0;
unsigned int pm2_5 = 0;
unsigned int pm10 = 0;
unsigned int measurement = 0;
unsigned int run_id=0;

//karta SD
const int chipSelect = PA4;

void refresh_id()
{
  run_id = EEPROM.read(0);
  Serial.print("Starting session id:");
  Serial.print(++run_id);
  EEPROM.write(0, run_id);
}

void setup()
{
 
  pinMode(PC13, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(115200); //serial for communication with PC
  Serial2.begin(9600); //serial for communication with Raspberry Pi
  Serial3.begin(9600); //serial for receiving data from PMS3003
  pms3003.begin(Serial3);

  while (!Serial && !Serial3 && !Serial2) // wait for serial ports to set up

  refresh_id();

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect))
  {
      Serial.println("Card failed, or not present");
  }
  else Serial.println("card initialized.");
}


void loop()
{
    String data_frame;
    delay(200);
    
    if(pms3003.measure()==true)
    {
        Serial.println("wynik pomiaru:");
        
        Serial.print("pm10:");
        Serial.println(pms3003.get_pm10());
      
        Serial.print("pm2.5:");
        Serial.println(pms3003.get_pm2_5());
      
        Serial.print("pm1:");
        Serial.println(pms3003.get_pm1());

        //sd_dump(data_frame);
    }

    //Serial.print("id sesji:");
    //Serial.println(run_id);
    
}

void sd_dump(String data_frame)
{
    File dataFile = SD.open("log.txt", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile)
    {
        dataFile.println(data_frame);
        dataFile.close();
        Serial.println("data write success");
    }
    // if the file isn't open, pop up an error:
    else
    {
        Serial.println("error opening log.txt");
    }
}


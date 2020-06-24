#include "Arduino.h"

class Pms3003
{
public:
    Pms3003();
	~Pms3003();
    bool measure();
    void begin(HardwareSerial& serial);

    int get_pm1();
    int get_pm2_5();
    int get_pm10();
	int get_pm1_tsi();
    int get_pm2_5_tsi();
    int get_pm10_tsi();

private:
	HardwareSerial *_serial;
	
    unsigned int pm1=0;
    unsigned int pm2_5=0;
    unsigned int pm10=0;
	unsigned int pm1_tsi=0;
    unsigned int pm2_5_tsi=0;
    unsigned int pm10_tsi=0;
	
	unsigned int checksum=0;
	
};

extern Pms3003 pms3003;

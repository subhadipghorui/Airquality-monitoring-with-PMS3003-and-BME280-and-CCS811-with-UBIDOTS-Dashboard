#include "pms3003.h"

Pms3003::Pms3003()
{
    pm1=0;
    pm2_5=0;
    pm10=0;
	pm1_tsi=0;
    pm2_5_tsi=0;
    pm10_tsi=0;
}

Pms3003::~Pms3003()
{
	delete _serial;
}

void Pms3003::begin(HardwareSerial &serial)
{
	_serial=&serial;
}

bool Pms3003::measure()
{
	/**
	 *  Function that tries to read pms3003 frames from _serial
	 *  To avoid UART buffer overflow it should be called at least every 200ms
	 */
	
	this->checksum=0;
	byte buffer[24];
	uint8_t attempts = 0;
	
	//if there was no start byte after 100 attempts, return false (fail)
	//this is to avoid infinite loop 
    while (_serial->available() && ++attempts < 100)
    {
		
		//discard data until start byte (0x42) comes 
		if(_serial->peek() != 0x42)
		{
			_serial->read(); //flush any byte that is not 0x42
			continue;
		}
		else 	
		{
			//A frame begining byte detected (0x42)
			if(_serial->available() < 24) delay(30); // give a chance for all bits to come
		}
		
		if(_serial->available() >= 24)
		{
			_serial->readBytes(buffer,24);
		}
		//if there is less than 24 bytes in buffer after 30ms something must have happened, abort
		else return false;
		
		for(int i=0;i<22;i++)
		{
			this->checksum+=buffer[i];
		}
		
		if(this->checksum != buffer[22]*256+buffer[23]) return false;
		
		
		/*
		//For debugging purposes you can display frame
		for(int i=0;i<24;i++)
		{
			this->checksum+=buffer[i];
			Serial.print(buffer[i]);
			Serial.print(' ');
		}
		

		Serial.println();
		
		*/
		
		this->pm1_tsi=buffer[4]*256+buffer[5];
		this->pm2_5_tsi=buffer[6]*256+buffer[7];
		this->pm10_tsi=buffer[8]*256+buffer[9];
		
		this->pm1=buffer[10]*256+buffer[11];
		this->pm2_5=buffer[12]*256+buffer[13];
		this->pm10=buffer[14]*256+buffer[15];
		
		return true;
    }
	
	return false;
}

int Pms3003::get_pm1()
    {
        return this->pm1;
    }

int Pms3003::get_pm2_5()
    {
        return this->pm2_5;
    }

int Pms3003::get_pm10()
    {
        return this->pm10;
    }
	
int Pms3003::get_pm1_tsi()
    {
        return this->pm1_tsi;
    }

int Pms3003::get_pm2_5_tsi()
    {
        return this->pm2_5_tsi;
    }

int Pms3003::get_pm10_tsi()
    {
        return this->pm10_tsi;
    }

	
Pms3003 pms3003; //create an instance of pms3003


#include "AsservStream_uartDecoder.h"
#include <cstdio>
#include <functional>

#define CALL_MEMBER(object,ptrToMember)  ((object).*(ptrToMember))


AsservStream_uartDecoder::AsservStream_uartDecoder()
{
	currentState = &AsservStream_uartDecoder::synchroLookUp;
}


void AsservStream_uartDecoder::processBytes(uint8_t *buffer, unsigned int nbBytes)
{
	for(int i=0; i<nbBytes; i++)
		CALL_MEMBER(*this,currentState)(buffer[i]);
}


void AsservStream_uartDecoder::synchroLookUp(uint8_t byte)
{
	constexpr uint32_t synchroWord = 0xDEADBEEF;
	static int nbSynchroByteFound = 0;

	constexpr uint32_t synchroWord_config = 0xCAFEDECA;
	static int nbSynchroConfigByteFound = 0;

    if( byte == ((uint8_t*)&synchroWord)[nbSynchroByteFound] )
    {
    	nbSynchroByteFound++;
    }
    else if( byte == ((uint8_t*)&synchroWord_config)[nbSynchroConfigByteFound] )
    {
    	nbSynchroConfigByteFound++;
    }
    else
    {
    	isCurrentSampleValid = false;
    	nbSynchroByteFound = 0;
    	nbSynchroConfigByteFound = 0;
    	printf("drop..\n");
    }

   if( nbSynchroByteFound == sizeof(synchroWord) )  // Synchro found !
   {
		nbSynchroByteFound = 0;
		nbSynchroConfigByteFound = 0;
		currentState =  &AsservStream_uartDecoder::getRemainingData;

		if( isCurrentSampleValid)
			decodedSampleQueue.push(currentDecodedSample);
   }


   if( nbSynchroConfigByteFound == sizeof(synchroWord_config) )  // Synchro found !
   {
		nbSynchroByteFound = 0;
		nbSynchroConfigByteFound = 0;
		currentState =  &AsservStream_uartDecoder::getRemainingConfig;


		if( isCurrentSampleValid)
			decodedSampleQueue.push(currentDecodedSample);
   }

}

void AsservStream_uartDecoder::getRemainingData(uint8_t byte)
{
    static int nbByteRead = 0;
    static uint8_t *currentDecodedSamplePtr = (uint8_t*)&currentDecodedSample;
    currentDecodedSamplePtr[nbByteRead++] = byte;

    if(nbByteRead == sizeof(currentDecodedSample))
    {   
        nbByteRead = 0;
		currentState =  &AsservStream_uartDecoder::synchroLookUp;
		isCurrentSampleValid = true;
	}
}

void AsservStream_uartDecoder::getRemainingConfig(uint8_t byte)
{
    static int nbByteRead = 0;
    static int nbByteToRead = 0;

    if( nbByteToRead == 0 )
    {
    	nbByteToRead = byte;
    }
    else
    {
    	configBuffer[nbByteRead++] = byte;
    	if( nbByteRead == nbByteToRead)
    	{
    		nbValues = nbByteRead;
    		configAvailable = true;
        	printf("Received %d bytes of config\n", nbByteRead);

    		nbByteRead = 0;
    		nbByteToRead = 0;
    		currentState =  &AsservStream_uartDecoder::synchroLookUp;
    	}
    }

}

bool AsservStream_uartDecoder::getDecodedSample(UsbStreamSample *sample)
{
	if(decodedSampleQueue.empty() || sample == NULL )
		return false;
	*sample = decodedSampleQueue.front();
	decodedSampleQueue.pop();
	return true;
}

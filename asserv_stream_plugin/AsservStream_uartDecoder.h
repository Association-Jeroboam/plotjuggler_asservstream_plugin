#ifndef ASSERV_STREAM_DECODER_H
#define ASSERV_STREAM_DECODER_H
#include <cstdint>
#include <queue>


typedef struct
{
	uint32_t timestamp;
	float value1;
	float value2;
	float value3;
	float value4;
}  __attribute__((packed)) UsbStreamSample;


class  AsservStream_uartDecoder
{
public:

	AsservStream_uartDecoder();
    virtual ~AsservStream_uartDecoder(){};
	void processBytes(uint8_t *buffer, unsigned int nbBytes);

	bool getDecodedSample(UsbStreamSample *sample);

	uint8_t configBuffer[512];
	int nbValues;
	bool configAvailable = false;

private:
	UsbStreamSample currentDecodedSample;
	bool isCurrentSampleValid = false;

	std::queue<UsbStreamSample> decodedSampleQueue;

	void synchroLookUp(uint8_t byte);
	void getRemainingData(uint8_t byte);
	void getRemainingConfig(uint8_t byte);

	typedef void (AsservStream_uartDecoder::*stateFunction)(uint8_t byte);
	stateFunction currentState;
};

#endif


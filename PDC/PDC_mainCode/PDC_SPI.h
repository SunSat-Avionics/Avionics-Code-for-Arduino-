/* all devices on the bus are happy with 10MHz clock */
#define CLOCK_RATE 10000000

void readSPI(uint8_t deviceSelect, uint8_t registerSelect, uint8_t numBytes, uint8_t *result);
void writeSPI(uint8_t deviceSelect, uint8_t registerSelect, uint8_t data);

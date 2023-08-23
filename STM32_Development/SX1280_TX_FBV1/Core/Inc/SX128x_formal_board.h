#include "SX128x.hpp"
#include "main.h"

#define SOPHIA_V1

class SX128x_formal_board : public SX128x
{
private:

    void HalGpioWrite(GpioPinFunction_t func, uint8_t value);
    void HalSpiTransfer(uint8_t *buffer_in, const uint8_t *buffer_out, uint16_t size);
    uint8_t HalGpioRead(GpioPinFunction_t func);

public:
    // common setting
    void CommonTranceiverSetting();

    // tx operation
    void TxSetting();
    uint8_t TxBlocking(uint8_t* data_out, uint8_t len);

    // set interrupt
    void InterruptSetting();
    void InterruptClearing();
};

#include "SX128x.hpp"
#include "main.h"

//#define SOPHIA_V1
#define NUCLEO_L476

#define PACKET_SIZE 8
#define FIFO_SIZE 8
#define IS_TX

class SX128x_formal_board : public SX128x
{
private:
#ifdef IS_TX
    uint8_t tx_fifo[FIFO_SIZE][8] = {};
    uint8_t* tx_sprt = tx_fifo[0];
    uint8_t* tx_eprt = tx_fifo[0];
    int8_t tx_length = 0;
    uint8_t tx_activated = 0;
#endif

    uint32_t RfFrequency = 2400000000;
    ModulationParams_t mparams;
    PacketParams_t pparams;

    void HalGpioWrite(GpioPinFunction_t func, uint8_t value);
    void HalSpiTransfer(uint8_t *buffer_in, const uint8_t *buffer_out, uint16_t size);
    uint8_t HalGpioRead(GpioPinFunction_t func);
    void HalSpiTransferDelay(uint8_t *buffer_in, const uint8_t *buffer_out, uint16_t size);

public:
    // constructor
    SX128x_formal_board();

    // common setting
    void CommonTransceiverSetting();

    // tx operation
    void TxSetting();
    uint8_t TxBlocking(uint8_t* data_out, uint8_t len);

    // set interrupt
    void InterruptSetting();
    void InterruptClearing();

    // initializer
    uint8_t Init(); // including common transceiver setting

    // put packet
    uint8_t PutPacket(uint8_t* in);

    // get length
    uint8_t GetLength();

    // interrupt recursive
    void tx_recursion();
};

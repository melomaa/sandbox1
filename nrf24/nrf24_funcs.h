/*
 * nrf24_funcs.h
 *
 *  Created on: Feb 28, 2015
 *
 */

#ifndef NRF24_FUNCS_H_
#define NRF24_FUNCS_H_

#include "nrf24_base.h"

/**
 * Empty the receive buffer
 *
 * @param	ts	Pointer to nrf24 data structure
 *
 * @return Current value of status register
 */
uint8_t flush_rx(struct nrf24_chip *ts);

/**
 * Empty the transmit buffer. This is generally not required in standard operation.
 * May be required in specific cases after stopListening() , if operating at 250KBPS data rate.
 *
 * @param	ts	Pointer to nrf24 data structure
 *
 * @return Current value of status register
 */
uint8_t flush_tx(struct nrf24_chip *ts);

/**
 * Enter low-power mode
 *
 * To return to normal power mode, call powerUp().
 *
 * @note After calling startListening(), a basic radio will consume about 13.5mA
 * at max PA level.
 * During active transmission, the radio will consume about 11.5mA, but this will
 * be reduced to 26uA (.026mA) between sending.
 * In full powerDown mode, the radio will consume approximately 900nA (.0009mA)
 *
 * @param	ts	Pointer to nrf24 data structure
 */
void powerUp(struct nrf24_chip *ts);

/**
 * Leave low-power mode - required for normal radio operation after calling powerDown()
 *
 * To return to low power mode, call powerDown().
 * @note This will take up to 5ms for maximum compatibility
 *
 * @param	ts	Pointer to nrf24 data structure
 */
void powerDown(struct nrf24_chip *ts);

/**
 * Turn on or off the special features of the chip
 *
 * The chip has certain 'features' which are only available when the 'features'
 * are enabled.  See the datasheet for details.
 *
 * @param	ts	Pointer to nrf24 data structure
 */
void toggle_features(struct nrf24_chip *ts);

/**
 * Enable custom payloads on the acknowledge packets
 *
 * Ack payloads are a handy way to return data back to senders without
 * manually changing the radio modes on both units.
 *
 * @note Ack payloads are dynamic payloads. This only works on pipes 0&1 by default. Call
 * enableDynamicPayloads() to enable on all pipes.
 *
 * @param	ts	Pointer to nrf24 data structure
 */
void enableAckPayload(struct nrf24_chip *ts);

/**
 * Enable dynamically-sized payloads
 *
 * This way you don't always have to send large packets just to send them
 * once in a while.  This enables dynamic payloads on ALL pipes.
 *
 * @param	ts	Pointer to nrf24 data structure
 */
void enableDynamicPayloads(struct nrf24_chip *ts);

/**
 * Enable or disable auto-acknowlede packets
 *
 * This is enabled by default, so it's only needed if you want to turn
 * it off for some reason.
 *
 * @param	ts	Pointer to nrf24 data structure
 * @param enable Whether to enable (true) or disable (false) auto-acks
 */
void setAutoAck(struct nrf24_chip *ts, bool enable);

/**
 * Enable or disable auto-acknowlede packets on a per pipeline basis.
 *
 * AA is enabled by default, so it's only needed if you want to turn
 * it off/on for some reason on a per pipeline basis.
 *
 * @param	ts	Pointer to nrf24 data structure
 * @param pipe Which pipeline to modify
 * @param enable Whether to enable (true) or disable (false) auto-acks
 */
void setAutoAckPipe(struct nrf24_chip *ts, uint8_t pipe, bool enable );


/**
 * Set RF communication channel
 *
 * @param	ts	Pointer to nrf24 data structure
 * @param channel Which RF channel to communicate on, 0-127
 */
void setChannel(struct nrf24_chip *ts, uint8_t channel);

/**
 * Set the CRC length
 * <br>CRC checking cannot be disabled if auto-ack is enabled
 *
 * @param	ts	Pointer to nrf24 data structure
 * @param length RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
 */
void setCRCLength(struct nrf24_chip *ts, rf24_crclength_e length);

/**
 * Get the CRC length
 * <br>CRC checking cannot be disabled if auto-ack is enabled
 *
 * @param	ts	Pointer to nrf24 data structure
 *
 * @return RF24_DISABLED if disabled or RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
 */
rf24_crclength_e getCRCLength(struct nrf24_chip *ts);

/**
 * Set the transmission data rate
 *
 * @warning setting RF24_250KBPS will fail for non-plus units
 *
 * @param	ts	Pointer to nrf24 data structure
 * @param speed RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
 *
 * @return true if the change was successful
 */
bool setDataRate(struct nrf24_chip *ts, rf24_datarate_e speed);

/**
 * Set Power Amplifier (PA) level to one of four levels:
 * RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
 *
 * The power levels correspond to the following output levels respectively:
 * NRF24L01: -18dBm, -12dBm,-6dBM, and 0dBm
 *
 * SI24R1: -6dBm, 0dBm, 3dBM, and 7dBm.
 *
 * @param	ts	Pointer to nrf24 data structure
 * @param level Desired PA level.
 */
void setPALevel(struct nrf24_chip *ts, rf24_pa_dbm_e level);

/**
 * Set Static Payload Size
 *
 * This implementation uses a pre-stablished fixed payload size for all
 * transmissions.  If this method is never called, the driver will always
 * transmit the maximum payload size (32 bytes), no matter how much
 * was sent to write().
 *
 * @todo Implement variable-sized payloads feature
 *
 * @param	ts	Pointer to nrf24 data structure
 * @param size The number of bytes in the payload
 */
void setPayloadSize(struct nrf24_chip *ts, uint8_t size);

/**
 * Set the number and delay of retries upon failed submit
 *
 * @param	ts	Pointer to nrf24 data structure
 * @param delay How long to wait between each retry, in multiples of 250us,
 * max is 15.  0 means 250us, 15 means 4000us.
 * @param count How many retries before giving up, max 15
 */
void setRetries(struct nrf24_chip *ts, uint8_t delay, uint8_t count);


/**
 * Open a pipe for writing
 *
 * Addresses are 40-bit hex values, e.g.:
 *
 * @param	ts	Pointer to nrf24 data structure
 * @param value The 40-bit address of the pipe to open.
 */
void openWritingPipe(struct nrf24_chip *ts, uint64_t value);

/**
 * Open a pipe for reading
 *
 * @warning Pipes 1-5 should share the first 32 bits.
 * Only the least significant byte should be unique, e.g.
 *
 * @warning Pipe 0 is also used by the writing pipe.  So if you open
 * pipe 0 for reading, and then startListening(), it will overwrite the
 * writing pipe.  Ergo, do an openWritingPipe() again before write().
 *
 * @param	ts	Pointer to nrf24 data structure
 * @param child Which pipe# to open, 0-5.
 * @param address The 40-bit address of the pipe to open.
 */
void openReadingPipe(struct nrf24_chip *ts, uint8_t child, uint64_t address);

/**
 * Start listening on the pipes opened for reading.
 *
 * @param	ts	Pointer to nrf24 data structure
 */
void startListening(struct nrf24_chip *ts);

/**
 * Stop listening for incoming messages, and switch to transmit mode.
 *
 * @param	ts	Pointer to nrf24 data structure
 */
void stopListening(struct nrf24_chip *ts);

#endif /* NRF24_FUNCS_H_ */

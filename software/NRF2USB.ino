// ===================================================================================
// Project:   NRF2USB - nRF24L01+ 2.4GHz Transceiver USB Stick
// Version:   v1.1
// Year:      2021
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// NRF2USB is a simple development tool for wireless applications based on the
// nRF24L01+ 2.4GHz transceiver module. It provides a serial interface for
// communication with the module via USB. The CH330N (or CH340N) USB to serial
// chip can also function as a SerialUPDI programmer for the integrated ATtiny814
// (or compatible), so that no external programming device is necessary.
//
// Wiring:
// -------
//                            +-\/-+
//                      Vcc  1|°   |14  GND
//         --- !SS AIN4 PA4  2|    |13  PA3 AIN3 SCK ---- NRF SCK
//         ------- AIN5 PA5  3|    |12  PA2 AIN2 MISO --- NRF MISO
// NRF IRQ --- DAC AIN6 PA6  4|    |11  PA1 AIN1 MOSI --- NRF MOSI
//     LED ------- AIN7 PA7  5|    |10  PA0 AIN0 UPDI --- UPDI
// USB RXD -------- RXD PB3  6|    |9   PB0 AIN11 SCL --- NRF CSN
// USB TXD ---------TXD PB2  7|    |8   PB1 AIN10 SDA --- NRF CE
//                            +----+
//
// Compilation Settings:
// ---------------------
// Core:    megaTinyCore (https://github.com/SpenceKonde/megaTinyCore)
// Board:   ATtiny1614/1604/814/804/414/404/214/204
// Chip:    Choose the chip you use
// Clock:   10 MHz internal
//
// Leave the rest on default settings. Select "SerialUPDI" as programmer in the
// Arduino IDE and set the selector switch on the board to "UPDI". Don't forget
// to "Burn bootloader"! Compile and upload the code.
//
// Since the ATtiny is operated with 3.3V, its clock frequency should not be higher
// than 10 MHz.
//
// No Arduino core functions or libraries are used. Use the makefile to compile 
// and upload without Arduino IDE.
//
// Fuse Settings: 0:0x00 1:0x00 2:0x02 4:0x00 5:0xC5 6:0x04 7:0x00 8:0x00
//
// Operating Instructions:
// -----------------------
// Plug the device into a USB port and set the selector switch on the board to "UART".
// Use a serial monitor with 230400 BAUD. Enter the text to be sent, terminated
// with a Newline (NL or '\ n'). A string that begins with an exclamation mark ('!')
// is recognized as a command. The command is given by the letter following the
// exclamation mark. Command arguments are appended as bytes in 2-digit hexadecimal
// directly after the command. The following commands can be used to set the NRF:
//
// cmd  description       example         example description
// -----------------------------------------------------------------------------------
//  c   set channel       !c2A            set channel to 0x2A (0x00 - 0x7F)
//  t   set TX address    !t7B271F1F1F    addresses are 5 bytes, LSB first
//  r   set RX address    !r41C355AA55    addresses are 5 bytes, LSB first
//  s   set speed         !s02            data rate (00:250kbps, 01:1Mbps, 02:2Mbps)
//
// Enter just the exclamation mark ('!') for the actual NRF settings to be printed
// in the serial monitor. The selected settings are saved in the EEPROM and are
// retained even after a restart.


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================

// Libraries
#include <avr/io.h>             // for GPIO
#include <avr/eeprom.h>         // to store data in EEPROM
#include <avr/interrupt.h>      // for interrupts
#include <util/delay.h>         // for delays

// Pin definitions
#define PIN_MOSI      PA1       // NRF SPI master out slave in
#define PIN_MISO      PA2       // NRF SPI master in slave out
#define PIN_SCK       PA3       // NRF SPI serial clock
#define PIN_IRQ       PA6       // NRF interrupt pin
#define PIN_LED       PA7       // pin connected to builtin LED
#define PIN_CSN       PB0       // NRF SPI slave select pin
#define PIN_CE        PB1       // NRF cable enable pin
#define PIN_TXD       PB2       // UART TX pin connected to usb-to-serial converter
#define PIN_RXD       PB3       // UART RX pin connected to usb-to-serial converter

// Configuration parameters
#define UART_BAUD     230400    // UART baud rate (max 1/8 of F_CPU)
#define UART_BUF_LEN  64        // UART RX buffer length (must be a power of two, max 256)
#define NRF_PAYLOAD   32        // NRF max payload (1-32)
#define NRF_CONFIG    0x0C      // CRC scheme, 0x08:8bit, 0x0C:16bit
#define EEPROM_IDENT  0xA96C    // to identify if EEPROM was written by this program
#define CMD_IDENT     '!'       // command string identifier

// Pin manipulation macros
enum {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PB2, PB3};    // enumerate pin designators
#define pinOutput(x)      (&VPORTA.DIR)[((x)&8)>>1] |=  (1<<((x)&7))  // set pin to OUTPUT
#define pinLow(x)         (&VPORTA.OUT)[((x)&8)>>1] &= ~(1<<((x)&7))  // set pin to LOW
#define pinHigh(x)        (&VPORTA.OUT)[((x)&8)>>1] |=  (1<<((x)&7))  // set pin to HIGH
#define pinToggle(x)      (&VPORTA.IN )[((x)&8)>>1] |=  (1<<((x)&7))  // TOGGLE pin
#define pinRead(x)        ((&VPORTA.IN)[((x)&8)>>1] &   (1<<((x)&7))) // READ pin

// ===================================================================================
// UART Implementation (8N1, RX ring buffer with interrupt, calibrated)
// ===================================================================================

// UART definitions and macros
#define UART_BAUD_RATE    8.0 * F_CPU / UART_BAUD + 0.5
#define UART_ready()      (USART0.STATUS & USART_DREIF_bm)
#define UART_available()  (UART_RX_head != UART_RX_tail)

// UART RX buffer and pointer
volatile uint8_t UART_RX_buf[UART_BUF_LEN];     // RX ring buffer
volatile uint8_t UART_RX_head = 0;              // RX buffer pointer for writing
         uint8_t UART_RX_tail = 0;              // RX buffer pointer for reading

// UART init
void UART_init(void) {
  pinOutput(PIN_TXD);                           // set TXD pin to output
  int8_t sigrow_val = SIGROW_OSC20ERR5V;        // get calibration value
  int32_t baud_setting = UART_BAUD_RATE;        // calculate baud register value ...
  baud_setting *= (1024 + sigrow_val);          // ... with error compensation ...
  baud_setting /= 1024;                         // ... for 5V and 20MHz oscillator
  USART0.BAUD  = (uint16_t)baud_setting;        // set BAUD
  USART0.CTRLA = USART_RXCIE_bm;                // enable RX interrupt
  USART0.CTRLB = USART_RXEN_bm                  // enable RX
               | USART_TXEN_bm                  // enable TX
               | USART_RXMODE_CLK2X_gc;         // double speed
}

// UART transmit data byte
void UART_write(uint8_t data) {
  while(!UART_ready());                         // wait until ready for next data
  USART0.TXDATAL = data;                        // send data byte
}

// UART read data byte from RX buffer
uint8_t UART_read(void) {
  while(!UART_available());                     // wait for data to be received
  UART_RX_tail &= (UART_BUF_LEN - 1);           // limit pointer
  return UART_RX_buf[UART_RX_tail++];           // read and return data byte
}

// UART RXC interrupt service routine
ISR(USART0_RXC_vect) {
  UART_RX_head &= (UART_BUF_LEN - 1);           // limit pointer
  UART_RX_buf[UART_RX_head++] = USART0.RXDATAL; // write received byte to buffer
}

// ===================================================================================
// String Conversions
// ===================================================================================

// Print string via UART
void UART_print(const uint8_t *str) {
  while(*str) UART_write(*str++);
}

// Print string via UART with new line
void UART_println(const uint8_t *str) {
  UART_print(str);
  UART_write('\n');
}

// Convert byte nibble into hex character and print via UART
void UART_printNibble(uint8_t nibble) {
  (nibble <= 9) ? (nibble += '0') : (nibble += ('A' - 10));
  UART_write(nibble);
}

// Convert byte into hex string and print via UART
void UART_printByte(uint8_t value) {
  UART_printNibble (value >> 4);
  UART_printNibble (value & 0x0F);
}

// Convert an array of bytes into hex string and print via UART
void UART_printBytes(uint8_t *ptr, uint8_t len) {
  while(len--) UART_printByte(*ptr++);
}

// Convert character representing a hex nibble into 4-bit value
uint8_t hexDigit(uint8_t c) {
  if     ((c >= '0') && (c <= '9')) return(c - '0');
  else if((c >= 'a') && (c <= 'f')) return(c - 'a' + 10);
  else if((c >= 'A') && (c <= 'F')) return(c - 'A' + 10); 
  return 0;
}

// Convert string containing a hex byte into 8-bit value
uint8_t hexByte(uint8_t *ptr) {
  return((hexDigit(*ptr++) << 4) + hexDigit(*ptr));
}

// Convert string containing 5 hex bytes into address array
void hexAddress(uint8_t *sptr, uint8_t *aptr) {
  for(uint8_t i=5; i; i--) {
    *aptr++ = hexByte(sptr);
    sptr += 2;
  }
}

// ===================================================================================
// SPI Master Implementation
// ===================================================================================

// SPI init
void SPI_init(void) {
  pinOutput(PIN_MOSI);                          // set MOSI pin as output
  pinOutput(PIN_SCK);                           // set SCK pin as output
  SPI0.CTRLA = SPI_CLK2X_bm                     // double speed
             | SPI_ENABLE_bm                    // enable SPI
             | SPI_MASTER_bm                    // master mode
             | SPI_PRESC_DIV4_gc;               // prescaler 4
  SPI0.CTRLB = SPI_SSD_bm;                      // disable SS line
}

// SPI transmit and receive a byte
uint8_t SPI_transfer(uint8_t data) {
  SPI0.DATA = data;                             // start exchanging data byte
  while(~SPI0.INTFLAGS & SPI_IF_bm);            // wait for transfer to complete
  return SPI0.DATA;                             // return received byte
}

// ===================================================================================
// nRF24L01+ Implementation - Definitions and Variables
// ===================================================================================

// NRF registers
#define NRF_REG_CONFIG        0x00              // configuration register
#define NRF_REG_RF_CH         0x05              // RF frequency channel
#define NRF_REG_RF_SETUP      0x06              // RF setup register
#define NRF_REG_STATUS        0x07              // status register
#define NRF_REG_RX_ADDR_P0    0x0A              // RX address pipe 0
#define NRF_REG_RX_ADDR_P1    0x0B              // RX address pipe 1
#define NRF_REG_TX_ADDR       0x10              // TX address
#define NRF_REG_FIFO_STATUS   0x17              // FIFO status register
#define NRF_REG_DYNPD         0x1C              // enable dynamic payload length
#define NRF_REG_FEATURE       0x1D              // feature

// NRF commands
#define NRF_CMD_R_RX_PL_WID   0x60              // read RX payload length
#define NRF_CMD_R_RX_PAYLOAD  0x61              // read RX payload
#define NRF_CMD_W_TX_PAYLOAD  0xA0              // write TX payload
#define NRF_CMD_FLUSH_TX      0xE1              // flush TX FIFO
#define NRF_CMD_FLUSH_RX      0xE2              // flush RX FIFO

// NRF global variables
uint8_t NRF_channel = 0x02;                     // channel (0x00 - 0x7F)
uint8_t NRF_speed   = 0;                        // 0:250kbps, 1:1Mbps, 2:2Mbps
uint8_t NRF_tx_buffer[NRF_PAYLOAD];             // transmit buffer
uint8_t NRF_tx_addr[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
uint8_t NRF_rx_addr[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
const uint8_t NRF_SETUP[] = {0x26, 0x06, 0x0E};
const uint8_t* NRF_STR[]  = {"250k", "1M", "2M"};

// ===================================================================================
// nRF24L01+ Implementation - SPI Communication Functions
// ===================================================================================

// NRF setup the pins
void NRF_init(void) {
  pinHigh(PIN_CSN);
  pinOutput(PIN_CE);
  pinOutput(PIN_CSN);
}

// NRF send a command
void NRF_writeCommand(uint8_t cmd) {
  pinLow(PIN_CSN);
  SPI_transfer(cmd);
  pinHigh(PIN_CSN);
}

// NRF write one byte into the specified register
void NRF_writeRegister(uint8_t reg, uint8_t value) {
  pinLow(PIN_CSN);
  SPI_transfer(reg + 0x20);
  SPI_transfer(value);
  pinHigh(PIN_CSN);
}

// NRF read one byte from the specified register
uint8_t NRF_readRegister(uint8_t reg) {
  pinLow(PIN_CSN);
  SPI_transfer(reg);
  uint8_t value = SPI_transfer(0);
  pinHigh(PIN_CSN);
  return value;
}

// NRF write an array of bytes into the specified registers
void NRF_writeBuffer(uint8_t reg, uint8_t *buf, uint8_t len) {
  if(reg < 0x20) reg += 0x20;
  pinLow(PIN_CSN);
  SPI_transfer(reg);
  while(len--) SPI_transfer(*buf++);
  pinHigh(PIN_CSN);
}

// NRF read an array of bytes from the specified registers
void NRF_readBuffer(uint8_t reg, uint8_t *buf, uint8_t len) {
  pinLow(PIN_CSN);
  SPI_transfer(reg);
  while(len--) *buf++ = SPI_transfer(0);
  pinHigh(PIN_CSN);
}

// ===================================================================================
// nRF24L01+ Implementation - Transceiver Functions
// ===================================================================================

// NRF switch to Power Down
void NRF_powerDown(void) {
  pinLow(PIN_CE);                                       // return to Standby-I
  NRF_writeRegister(NRF_REG_CONFIG, NRF_CONFIG | 0x00); // !PWR_UP
}

// NRF switch to RX mode
void NRF_powerRX(void) {
  pinLow(PIN_CE);                                       // return to Standby-I
  NRF_writeRegister(NRF_REG_CONFIG, NRF_CONFIG | 0x03); // PWR_UP + PRIM_RX
  pinHigh(PIN_CE);                                      // switch to RX Mode
}

// NRF switch to TX mode
void NRF_powerTX(void) {
  pinLow(PIN_CE);                                       // return to Standby-I
  NRF_writeRegister(NRF_REG_CONFIG, NRF_CONFIG | 0x02); // PWR_UP + !PRIM_RX
  pinHigh(PIN_CE);                                      // switch to TX Mode
}

// NRF configure
void NRF_configure(void) {
  pinLow(PIN_CE);                                       // leave active mode
  NRF_writeBuffer(NRF_REG_RX_ADDR_P1, NRF_rx_addr, 5);  // set RX address
  NRF_writeBuffer(NRF_REG_TX_ADDR,    NRF_tx_addr, 5);  // set TX address
  NRF_writeBuffer(NRF_REG_RX_ADDR_P0, NRF_tx_addr, 5);  // set TX address for auto-ACK
  NRF_writeRegister(NRF_REG_RF_CH, NRF_channel);        // set channel
  NRF_writeRegister(NRF_REG_RF_SETUP, NRF_SETUP[NRF_speed]); // set speed and power
  NRF_writeRegister(NRF_REG_FEATURE,  0x04);            // enable dynamic payload length
  NRF_writeRegister(NRF_REG_DYNPD,    0x3F);            // enable dynamic payload length
  NRF_writeCommand(NRF_CMD_FLUSH_RX);                   // flush RX FIFO
  NRF_powerRX();                                        // switch to RX Mode
}

// Check if data is available for reading
uint8_t NRF_available(void) {
  if(NRF_readRegister(NRF_REG_STATUS) & 0x40) return 1;
  return(!(NRF_readRegister(NRF_REG_FIFO_STATUS) & 0x01));
}

// Read payload bytes into data array; return payload length
uint8_t NRF_readData(uint8_t *data) {
  uint8_t len = NRF_readRegister(NRF_CMD_R_RX_PL_WID);  // read payload length
  NRF_readBuffer(NRF_CMD_R_RX_PAYLOAD, data, len);      // read payload
  NRF_writeRegister(NRF_REG_STATUS, 0x40);              // reset status register
  return len;                                           // return payload length
}

// Read payload bytes and send them via UART
void NRF_to_UART(void) {
  uint8_t len = NRF_readRegister(NRF_CMD_R_RX_PL_WID);  // read payload length
  pinLow(PIN_CSN);                                      // start SPI transfer
  SPI_transfer(NRF_CMD_R_RX_PAYLOAD);                   // read payload command
  while(len--) UART_write(SPI_transfer(0));             // transfer payload to UART
  pinHigh(PIN_CSN);                                     // stop SPI transfer
  NRF_writeRegister(NRF_REG_STATUS, 0x40);              // clear status flags
}

// Send a data package (max length 32)
void NRF_send(uint8_t *data, uint8_t len) {
  NRF_writeRegister(NRF_REG_STATUS, 0x30);              // clear status flags
  NRF_writeCommand(NRF_CMD_FLUSH_TX);                   // flush TX FIFO
  NRF_powerTX();                                        // switch to TX Mode; transmit
  NRF_writeBuffer(NRF_CMD_W_TX_PAYLOAD, data, len);     // write payload
  while(!(NRF_readRegister(NRF_REG_STATUS) & 0x30));    // wait until finished
  NRF_powerRX();                                        // return to listening
}

// Print the current NRF settings via UART
void NRF_printSettings(void) {
  UART_println("# NRF2USB Configuration:");
  UART_print  ("# RF channel: "); UART_printByte (NRF_channel); UART_write('\n');
  UART_print  ("# TX address: "); UART_printBytes(NRF_tx_addr, 5); UART_write('\n');
  UART_print  ("# RX address: "); UART_printBytes(NRF_rx_addr, 5); UART_write('\n');
  UART_print  ("# Data rate:  "); UART_print(NRF_STR[NRF_speed]); UART_println("bps");
}

// ===================================================================================
// EEPROM Implementation
// ===================================================================================

// EEPROM write user settings
void EEPROM_update(void) {
  eeprom_update_byte ((uint8_t*)2, NRF_channel);
  eeprom_update_byte ((uint8_t*)3, NRF_speed);
  eeprom_update_block((const void*)&NRF_tx_addr, (void*)4, 5);
  eeprom_update_block((const void*)&NRF_rx_addr, (void*)9, 5);
}

// EEPROM read user settings; if EEPROM values are invalid, write defaults
void EEPROM_get(void) {
  uint16_t identifier = eeprom_read_word((const uint16_t*)0);
  if (identifier == EEPROM_IDENT) {
    NRF_channel =  eeprom_read_byte((const uint8_t*)2);
    NRF_speed   =  eeprom_read_byte((const uint8_t*)3);
    eeprom_read_block((void*)&NRF_tx_addr, (const void*)4, 5);
    eeprom_read_block((void*)&NRF_rx_addr, (const void*)9, 5);
  }
  else {
    eeprom_update_word((uint16_t*)0, EEPROM_IDENT);
    EEPROM_update();
  }
}

// ===================================================================================
// Command Parser
// ===================================================================================

void parse(void) {
  uint8_t cmd = NRF_tx_buffer[1];                   // read the command
  switch(cmd) {                                     // what command?
    case 'c': NRF_channel = hexByte(NRF_tx_buffer + 2) & 0x7F;
              break;
    case 't': hexAddress(NRF_tx_buffer + 2, NRF_tx_addr);
              break;
    case 'r': hexAddress(NRF_tx_buffer + 2, NRF_rx_addr);
              break;
    case 's': NRF_speed = hexByte(NRF_tx_buffer + 2);
              if(NRF_speed > 2) NRF_speed = 2;
              break;
    default:  break;
  }
  NRF_configure();                                  // reconfigure the NRF
  NRF_printSettings();                              // print settings via UART
  EEPROM_update();                                  // update settings in EEPROM
}

// ===================================================================================
// Main Function
// ===================================================================================

int main(void) {
  // Setup MCU
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 1);           // set clock frequency to 10 MHz
  
  // Setup
  uint8_t tx_ptr = 0;                               // NRF TX buffer pointer
  pinOutput(PIN_LED);                               // set LED pin as output
  EEPROM_get();                                     // read user settings from EEPROM
  NRF_init();                                       // setup NRF
  SPI_init();                                       // setup SPI serial interface
  UART_init();                                      // setup UART serial interface
  NRF_configure();                                  // configure NRF
  sei();

  // Loop
  while(1) {    
    if(NRF_available()) {                           // something coming in via NRF?
      pinHigh(PIN_LED);                             // switch on LED
      NRF_to_UART();                                // send received payload via UART
    }

    if(UART_available()) {                          // something coming in via UART?
      uint8_t c = UART_read();                      // read the character ...
      NRF_tx_buffer[tx_ptr++] = c;                  // ... and write it to the buffer
      if((tx_ptr == NRF_PAYLOAD) || (c == '\n')) {  // buffer full or new line?
        if(NRF_tx_buffer[0] == CMD_IDENT) parse();  // is it a command? -> parse
        else {                                      // not a command?
          pinHigh(PIN_LED);                         // switch on LED
          NRF_send(NRF_tx_buffer, tx_ptr);          // send the buffer via NRF
        }
        tx_ptr = 0;                                 // reset buffer pointer
      }
    }

    pinLow(PIN_LED);                                // switch off LED
  }
}

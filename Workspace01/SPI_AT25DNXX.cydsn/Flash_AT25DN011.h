/**
 *  @file   Flash_AT25DN011.h
 *  @brief  flash AT25DN011 command definitions
 *  @author Rafael Dias Menezes <rd185189@ncr.com>
 *  @date   december/2017
 */
    
#ifndef _AT25DN011_H_
#define _AT25DN011_H_
    
#include "defs.h"
#include "sw_flash_spi_interface.h"
    
    
/**
 *  @typedef Flash_AT25DN011_CommandList
 *  @brief  define the commands supported by AT25DN011
 */
typedef enum 
{
    AT25DN011_Read_Array                        =   0x0B, 
    AT25DN011_Dual_Output_Read                  =   0x3B, 
    AT25DN011_Page_Erase                        =   0x81, 
    AT25DN011_Block_Erase_4_Kbytes              =   0x20,
    AT25DN011_Block_Erase_32_Kbytes             =   0x52,
    AT25DN011_Chip_Erase                        =   0x60,
    AT25DN011_Page_Program                      =   0x02,
    AT25DN011_Write_Enable                      =   0x06,
    AT25DN011_Write_Disable                     =   0x04,
    AT25DN011_Program_OTP_Security_Register     =   0x9B,
    AT25DN011_Read_OTP_Security_Register        =   0x77,
    AT25DN011_Read_Status_Register              =   0x05, 
    AT25DN011_Write_Status_Register1            =   0x01,
    AT25DN011_Write_Status_Register2            =   0x31,
    AT25DN011_Reset                             =   0xF0, 
    AT25DN011_Read_ManufID                      =   0x9F, 
    AT25DN011_Deep_PowerDown                    =   0xB9, 
    AT25DN011_Resume_Deep_PowerDown             =   0xAB, 
    AT25DN011_Ultra_Deep_Power_Down             =   0x79, 
}Flash_AT25DN011_CommandList;


typedef struct _Flash_AT25DN011_Status
{
    union
    {
        struct
        {
            BYTE reg1;
            BYTE reg2;
        };
        
        struct 
        {
            uint32_t BSY1:1;    // bit 0 byte 0
            uint32_t WEL:1;     // bit 1 byte 0
            uint32_t BP0:1;     // bit 2 byte 0
            uint32_t RES2:1;    // bit 3 byte 0
            uint32_t WPP:1;     // bit 4 byte 0
            uint32_t EPE:1;     // bit 5 byte 0
            uint32_t RES1:1;    // bit 6 byte 0
            uint32_t BPL:1;     // bit 7 byte 0
            
            //--
            
            uint32_t BSY2:1;    // bit 0 byte 1        
            uint32_t RES8:1;    // bit 1 byte 1
            uint32_t RES7:1;    // bit 2 byte 1
            uint32_t RES6:1;    // bit 3 byte 1
            uint32_t RSTE:1;    // bit 4 byte 1
            uint32_t RES5:1;    // bit 5 byte 1
            uint32_t RES4:1;    // bit 6 byte 1
            uint32_t RES3:1;    // bit 7 byte 1
        };
    };
}Flash_AT25DN011_Status_Reg;

#define AT25DN011_PAGE_SIZE     (256L)
#define AT25DN011_MEM_SIZE      (1024L*1024L/8)
//static uint8_t AT25DN011_buffer[AT25DN011_PAGE_SIZE];

/**
 *  @function   AT25DN011_ReadMISO
 *  @brief      read the MISO signal
 *  @param[in]  dev pointer to a SPI device 
 *  @returns    MISO signal state
 */
uint32_t    AT25DN011_ReadMISO(TSwSpi* dev);

/**
 *  @function   AT25DN011_WriteMOSI
 *  @brief      write bData on MOSI signal
 *  @param[in]  dev pointer to a SPI device 
 */
void        AT25DN011_WriteMOSI(TSwSpi* dev, BYTE bData);

/**
 *  @function   AT25DN011_WriteCS
 *  @brief      write bData on CS signal
 *  @param[in]  dev pointer to a SPI device 
 */
void        AT25DN011_WriteCS(TSwSpi* dev, BYTE bData);

/**
 *  @function   AT25DN011_WriteClockPin
 *  @brief      write bData on Clock signal
 *  @param[in]  dev pointer to a SPI device 
 */
void        AT25DN011_WriteClockPin(TSwSpi* dev, BYTE bData);

/**
 *  @function   AT25DN011_ReadClockPin
 *  @brief      read the clock signal
 *  @param[in]  dev pointer to a SPI device 
 */
uint32_t    AT25DN011_ReadClockPin(TSwSpi*);

/**
 *  @function   AT25DN011_Clock
 *  @brief      pulses the clock signal
 *  @param[in]  dev pointer to a SPI device 
 */
void        AT25DN011_Clock( TSwSpi* dev );

/**
 *  @function   AT25DN011_EnableCS
 *  @brief      asserts the chip select signal low
 *  @param[in]  dev pointer to a SPI device 
 */
void        AT25DN011_EnableCS( TSwSpi* dev );

/**
 *  @function   AT25DN011_DisableCS
 *  @brief      asserts the chip select signal high
 *  @param[in]  dev pointer to a SPI device 
 */
void        AT25DN011_DisableCS( TSwSpi* dev );

/**
 *  @function   AT25DN011_Send
 *  @brief      send bData across MOSI pin
 *  @param[in]  dev pointer to a SPI device 
 */
uint32_t    AT25DN011_Send( TSwSpi* dev, const BYTE bData );

/**
 *  @function   AT25DN011_Receive
 *  @brief      send bData across MOSI pin
 *  @param[in]  dev pointer to a SPI device 
 *  @param[in]  bData data to be sent
 */
uint32_t    AT25DN011_Receive( TSwSpi* dev, BYTE* pbData );

/**
 *  @brief      initialize the spi flash memory interface
 *  @param[in]  dev spi device to initialize
 *  @param[in]  mode communication mode. SPI_MODE0 and SPI_MODE2
 *  @returns    0 success
 *  @returns    > 0 error
 */
uint32_t    AT25DN011_init( TSwSpi* dev, TSpiMode  mode );

/**
 *  @brief      deinit the spi flash interface
 *  @param[in]  dev spi device to dinitialize
 *  @returns    0 success
 *  @returns    > 0 error
 */
uint32_t    AT25DN011_deinit( TSwSpi* dev );

/**
 *  @brief      write a buffer to SPI memory at dwAddr address
 *  @param[in]  dev         spi device to dinitialize
 *  @param[in]  dwAddr      address to write
 *  @param[in]  pbBuffer    buffer to write
 *  @param[in]  szSize      number of bytes to write
 *  @returns    0 success
 *  @returns    > 0 error
 */
uint32_t    AT25DN011_write( TSwSpi* dev, const DWORD dwAddr, BYTE* pbBuffer, const size_t szSize);

uint32_t    AT25DN011_read( TSwSpi* dev, const DWORD dwAddr, BYTE* pbBuffer, size_t szSize); 

uint32_t    AT25DN011_readid( TSwSpi* dev, BYTE* pbBuffer, const size_t szSize );

uint32_t    AT25DN011_wren( TSwSpi* dev );

uint32_t    AT25DN011_wrdi( TSwSpi* dev );

uint32_t    AT25DN011_rdst( TSwSpi* dev, void* reg );

#endif
    

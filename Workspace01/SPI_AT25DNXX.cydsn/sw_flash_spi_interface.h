/**
 *  @file   sw_flash_spi_interface.h
 *  @brief  software serial library struct definition
 *  @author Rafael Dias <rd185189@ncr.com>
 *  @date   Dez/2017
 */

#ifndef _SW_SPI_STRUCT_H_
#define _SW_SPI_STRUCT_H_

#include "defs.h"

    
/**
 *  @typedef    TSpiFlashModel
 *  @brief      inform the spi flash models
 */    
typedef enum 
{ 
    FlashAT25DN011, 
    FlashNONE,
}TSpiFlashModel;
    
/* Local module defines */
#define SPI_SW_ENABLE_CS    0L
#define SPI_SW_DISABLE_CS   1L    
    
/**
 *  @typedef    TSpiMode
 *  @brief      inform the spi communication models
 */    
typedef enum
{
    SPI_MODE0, 
    SPI_MODE1, 
    SPI_MODE2, 
    SPI_MODE3, 
} TSpiMode;

typedef struct TSwSpi TSwSpi;

struct TSwSpi
{
    uint32_t    cPol:1;
    uint32_t    cPha:1;
    
    TSpiMode    mode;
    
    uint32_t    (*ReadMISO)( TSwSpi* dev );
    void        (*WriteMOSI)( TSwSpi* dev, BYTE bData );
    void        (*WriteCS)( TSwSpi* dev, BYTE bState );
    void        (*WriteClockPin)( TSwSpi* dev, BYTE bState );
    uint32_t    (*ReadClockPin)( TSwSpi* dev );
    void        (*Clock)( TSwSpi* dev );
    void        (*EnableCS)( TSwSpi* dev );
    void        (*DisableCS)( TSwSpi* dev );
    uint32_t    (*Send)( TSwSpi* dev, const BYTE bData );
    uint32_t    (*Receive)( TSwSpi* dev, BYTE* pbData );
   
    uint32_t    (*init)( TSwSpi* dev, TSpiMode xMode );
    uint32_t    (*deinit)( TSwSpi* dev );
    uint32_t    (*write)( TSwSpi* dev, const DWORD dwAddr, BYTE* pbData, const size_t szSize );
    uint32_t    (*read)( TSwSpi* dev, const DWORD dwAddr, BYTE* pbData, size_t szSize ); 
    uint32_t    (*readid)( TSwSpi* dev, BYTE* pbBuffer, const size_t szSize );
    uint32_t    (*wren)( TSwSpi* dev );
    uint32_t    (*wrdi)( TSwSpi* dev );
    uint32_t    (*rdst)( TSwSpi* dev, void* reg );
};

/**
 *  @brief  GetFlashInterface
 *  @param  xSpiFlash   spi flash model definition
 *  @returns    valid SPI device object
 *  @returns    NULL on error
 */
TSwSpi* GetFlashInterface( const TSpiFlashModel xSpiFlash );


#endif 

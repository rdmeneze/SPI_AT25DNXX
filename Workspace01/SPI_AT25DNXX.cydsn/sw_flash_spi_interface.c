/**
 *  @file   sw_flash_spi_interface.c
 *  @brief  implementation of software flash interface 
 *  @author Rafael Dias <rd185189@ncr.com>
 *  @date   2017/12
 */
#include "Flash_AT25DN011.h"
#include "sw_flash_spi_interface.h"

/**
 *  local SPI flash interface
 */
static TSwSpi spiFlash[1];

/*****************************************************************************/

TSwSpi* GetFlashInterface( const TSpiFlashModel xSpiFlash )
{
    TSwSpi* oSpi = NULL;
    
    switch ( xSpiFlash )
    {
        case FlashAT25DN011:
        {
            /**!< HAL functions */
            spiFlash[0].ReadMISO        = AT25DN011_ReadMISO;
            spiFlash[0].WriteMOSI       = AT25DN011_WriteMOSI;
            spiFlash[0].WriteCS         = AT25DN011_WriteCS;
            spiFlash[0].WriteClockPin   = AT25DN011_WriteClockPin;
            spiFlash[0].ReadClockPin    = AT25DN011_ReadClockPin;
            spiFlash[0].Clock           = AT25DN011_Clock;
            spiFlash[0].EnableCS        = AT25DN011_EnableCS;
            spiFlash[0].DisableCS       = AT25DN011_DisableCS;
            spiFlash[0].Send            = AT25DN011_Send;
            spiFlash[0].Receive         = AT25DN011_Receive;
            
            /**|< initialization functions */
            spiFlash[0].init            = AT25DN011_init;
            spiFlash[0].deinit          = AT25DN011_deinit;
            spiFlash[0].readid          = AT25DN011_readid;
            
            /**|< work functions */
            spiFlash[0].write           = AT25DN011_write;
            spiFlash[0].read            = AT25DN011_read;

            spiFlash[0].wren            = AT25DN011_wren;
            spiFlash[0].wrdi            = AT25DN011_wrdi;
            spiFlash[0].rdst            = AT25DN011_rdst;    
            
            oSpi = &spiFlash[0];
            
            break;
        }
        
        case FlashNONE:
        default:
            break;
    }
    return oSpi;
}

/*****************************************************************************/


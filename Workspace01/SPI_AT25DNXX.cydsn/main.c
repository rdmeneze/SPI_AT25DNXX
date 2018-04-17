/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "sw_flash_spi_interface.h"

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    TSwSpi* xFlashSpi = GetFlashInterface( FlashAT25DN011 );
    xFlashSpi->init( xFlashSpi, SPI_MODE0 );    

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    struct 
    {
        uint8_t     bBuffer[16];
        uint32_t    dwAddr;
    } xData1, xData2, xData3;
    
    xData1.dwAddr = 230;
    memset( xData1.bBuffer, 0x43, sizeof( xData1.bBuffer ));
    xFlashSpi->write( xFlashSpi, xData1.dwAddr, xData1.bBuffer, sizeof( xData1.bBuffer ));
    memset( xData1.bBuffer, 0xEE, sizeof( xData1.bBuffer ));
    xFlashSpi->read( xFlashSpi, xData1.dwAddr, xData1.bBuffer, sizeof( xData1.bBuffer )); 

    xData2.dwAddr = xData1.dwAddr + sizeof( xData1.bBuffer );
    memset( xData2.bBuffer, 0x44, sizeof( xData2.bBuffer ));
    xFlashSpi->write( xFlashSpi, xData2.dwAddr, xData2.bBuffer, sizeof( xData2.bBuffer ));
    memset( xData2.bBuffer, 0xEE, sizeof( xData2.bBuffer ));
    xFlashSpi->read( xFlashSpi, xData2.dwAddr, xData2.bBuffer, sizeof( xData2.bBuffer )); 
    
    xData3.dwAddr = xData2.dwAddr + sizeof( xData2.bBuffer );
    memset( xData3.bBuffer, 0x45, sizeof( xData3.bBuffer ));
    xFlashSpi->write( xFlashSpi, xData3.dwAddr, xData3.bBuffer, sizeof( xData3.bBuffer ));
    memset( xData3.bBuffer, 0xEE, sizeof( xData3.bBuffer ));
    xFlashSpi->read( xFlashSpi, xData3.dwAddr, xData3.bBuffer, sizeof( xData3.bBuffer )); 

    uint8_t bBuffer1[ sizeof( xData1.bBuffer ) +  sizeof( xData2.bBuffer ) + sizeof( xData3.bBuffer )];
    xFlashSpi->read( xFlashSpi, xData1.dwAddr, bBuffer1, sizeof( bBuffer1 ));
    
    uint8_t bBuffer2[600];
    memset( bBuffer2, 0x66, sizeof( bBuffer2 ));
    xFlashSpi->write( xFlashSpi, xData3.dwAddr + 100, bBuffer2, sizeof( bBuffer2 ));
    
    memset( bBuffer2, 0x55, sizeof( bBuffer2 ));
    xFlashSpi->read( xFlashSpi, xData3.dwAddr + 100, bBuffer2, sizeof( bBuffer2 ));     
    

    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */

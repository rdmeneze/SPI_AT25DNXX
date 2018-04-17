/**
 *  @file   Flash_AT25DN011.c
 *  @brief  flash AT25DN011 command definitions. 
 *      Datasheet https://www.adestotech.com/wp-content/uploads/DS-AT25DN011_038.pdf
 *  @author Rafael Dias Menezes <rd185189@ncr.com>
 *  @date   december/2017
 */

#include "Flash_AT25DN011.h"
#include "Flash_CLK.h"
#include "Flash_SI.h"
#include "Flash_SO.h"
#include "Flash_CS.h"
#include <CyLib.h>

/*****************************************************************************/
/* AT25DN011 JEDEC ID */
const BYTE AT25DN011_JDECID[] = { 0x1f, 0x42, 0x00, 0x00 };

/*****************************************************************************/

#define Flash_SI_SetValue( x )                          \
            do                                          \
            {                                           \
                if ( (x) )                              \
                {                                       \
                    Flash_SI_DR |= Flash_SI_MASK;       \
                }                                       \
                else                                    \
                {                                       \
                    Flash_SI_DR &= ~Flash_SI_MASK;      \
                }                                       \
            }while(0)                                   \
                                                        
                                                        
#define Flash_SI_GetValue( )                            \
            ((Flash_SI_DR & Flash_SI_MASK) >> Flash_SI_SHIFT)

#define Flash_SO_SetValue( x )                          \
            do                                          \
            {                                           \
                if ( (x) )                              \
                {                                       \
                    Flash_SO_DR |= Flash_SO_MASK;       \
                }                                       \
                else                                    \
                {                                       \
                    Flash_SO_DR &= ~Flash_SO_MASK;      \
                }                                       \
            }while(0)                                   \
                                                        
                                                        
#define Flash_SO_GetValue( )                            \
            ((Flash_SO_DR & Flash_SO_MASK) >> Flash_SO_SHIFT)

#define Flash_CS_SetValue( x )                          \
            do                                          \
            {                                           \
                if ( (x) )                              \
                {                                       \
                    Flash_CS_DR |= Flash_CS_MASK;       \
                }                                       \
                else                                    \
                {                                       \
                    Flash_CS_DR &= ~Flash_CS_MASK;      \
                }                                       \
            }while(0)                                   \
                                                        
#define Flash_CS_GetValue( )                            \
            ((Flash_CS_DR & Flash_CS_MASK) >> Flash_CS_SHIFT)

#define Flash_CLK_SetValue( x )                         \
            do                                          \
            {                                           \
                if ( (x) )                              \
                {                                       \
                    Flash_CLK_DR |= Flash_CLK_MASK;     \
                }                                       \
                else                                    \
                {                                       \
                    Flash_CLK_DR &= ~Flash_CLK_MASK;    \
                }                                       \
            }while(0)                                   \

#define Flash_CLK_GetValue( )                           \
            ((Flash_CLK_DR & Flash_CLK_MASK) >> Flash_CLK_SHIFT)


/*****************************************************************************/

static void AT25DN011_Busy( TSwSpi* dev )
{
    do                                              
    {                                               
        Flash_AT25DN011_Status_Reg xStatusReg;      
        dev->rdst( dev, (void*)&xStatusReg );       
                                                    
        if ( xStatusReg.BSY1 )                      
            CyDelay( 1 );                           
        else                                        
            break;                                  
    }while( 1 );                                     
    return;
}

#define Flash_Busy() AT25DN011_Busy( dev )

/*****************************************************************************/

uint32_t AT25DN011_ReadMISO(TSwSpi* dev)
{
    uint32_t dwRet = (uint32_t)-1;
    if ( dev ) 
    {
        dwRet = Flash_SO_Read();
    }
    return dwRet;
}

/*****************************************************************************/

void AT25DN011_WriteMOSI(TSwSpi* dev, BYTE bState)
{
    if ( dev ) 
    {
        Flash_SI_SetValue( bState != 0 );
    }
    return;
}

/*****************************************************************************/

void AT25DN011_WriteCS(TSwSpi* dev, BYTE bState)
{
    if ( dev ) 
    {
        Flash_CS_SetValue( bState != 0 );
    }
    return;
}

/*****************************************************************************/

#define AT25DN011_CLOCK_DELAY 1

void AT25DN011_Clock(TSwSpi* dev)
{
    if ( dev )
    {
        CyDelayUs( AT25DN011_CLOCK_DELAY );
        Flash_CLK_SetValue( !Flash_CLK_GetValue() ); 
        CyDelayUs( AT25DN011_CLOCK_DELAY );
        Flash_CLK_SetValue( !Flash_CLK_GetValue() ); 
    }
    return;
}

/*****************************************************************************/

void AT25DN011_WriteClockPin(TSwSpi* dev, BYTE bData)
{
    if ( dev )
    {
        Flash_CLK_SetValue( bData );
    }
    return;
}

/*****************************************************************************/

uint32_t    AT25DN011_ReadClockPin(TSwSpi* dev)
{
    uint32_t dwRet = (uint32_t)-1;
    if ( dev )
    {
        dwRet = Flash_CLK_GetValue( );
    }
    return dwRet;    
}

/*****************************************************************************/

void AT25DN011_EnableCS(TSwSpi* dev)
{
    if ( dev )
    {
        Flash_CS_SetValue( SPI_SW_ENABLE_CS );
    }
}

/*****************************************************************************/

void AT25DN011_DisableCS(TSwSpi* dev)
{
    if ( dev )
    {
        Flash_CS_SetValue( SPI_SW_DISABLE_CS );
    }
}

/*****************************************************************************/

uint32_t AT25DN011_Send(TSwSpi* dev, const BYTE bData)
{
    uint32_t dwRet = ENXIO;
    
    if ( dev )
    {
        BYTE bBit = 0x80;
        
        dev->WriteClockPin( dev, dev->cPha );
        
        for ( size_t szBitCounter = 0; szBitCounter < 8; szBitCounter++ )
        {
            if ( !dev->cPha )
            {
                dev->WriteMOSI( dev, bData & bBit );
                dev->Clock( dev );
            }
            else
            {
                dev->Clock( dev );
                dev->WriteMOSI( dev, bData & bBit );
            }
            
            bBit >>= 1;
        }
        
        dwRet = 0;
    }
    return dwRet;
}

/*****************************************************************************/

uint32_t AT25DN011_Receive(TSwSpi* dev, BYTE* pbData)
{
    uint32_t dwRet = ENXIO;
    if ( dev )
    {
        if ( pbData )
        {
            uint8_t bData = 0;
            volatile int iCounter;
        
            for ( iCounter = 0; iCounter < 8; iCounter++ )
            {
                AT25DN011_WriteClockPin( dev, !AT25DN011_ReadClockPin( dev ));
                CyDelayUs( 10 );
                bData |= (dev->ReadMISO( dev ) << (7 - iCounter));
                AT25DN011_WriteClockPin( dev, !AT25DN011_ReadClockPin( dev ) );
                CyDelayUs( 10 );                
            }
            
            *pbData = bData;
            dwRet = 0;
        }
    }
    return dwRet;    
}

/*****************************************************************************/

uint32_t AT25DN011_PageErase(TSwSpi* dev, const DWORD dwAddr )
{
    uint32_t dwRet = ENXIO;
    if ( dev )
    {
        /* send the write enable command */
        AT25DN011_wren( dev );        
        
        DWORD dwPageAddr = dwAddr & 0x0001FF00;
        BYTE bCmdBuffer[4];
        
        bCmdBuffer[0] = AT25DN011_Page_Erase;
        bCmdBuffer[1] = (BYTE)( dwPageAddr >> 16 );
        bCmdBuffer[2] = (BYTE)( dwPageAddr >>  8 );
        bCmdBuffer[3] = (BYTE)( dwPageAddr );
        
        dev->EnableCS( dev );
        dev->Send( dev, bCmdBuffer[0] );
        dev->Send( dev, bCmdBuffer[1] );
        dev->Send( dev, bCmdBuffer[2] );
        dev->Send( dev, bCmdBuffer[3] );
        dev->DisableCS( dev );
        
        Flash_Busy();
        
        /* send the write disable command */
        AT25DN011_wrdi( dev );        
    }
    return dwRet;
}

/*****************************************************************************/

uint32_t AT25DN011_init(TSwSpi* dev, TSpiMode  mode )
{
    uint32_t dwRet = ENXIO;
    if ( dev )
    {
        dev->mode = mode;
        if ( SPI_MODE0 == dev->mode )
        {
            dev->cPha = 0;
            dev->cPol = 0;
        }
        
        if ( SPI_MODE2 == dev->mode )
        {
            dev->cPha = 1;
            dev->cPol = 0;            
        }
        
        if ( !dev->cPol )
        {
            dev->WriteClockPin( dev, 0 );
        }
        else
        {
            dev->WriteClockPin( dev, 1 );
        }

        dev->WriteCS( dev, SPI_SW_DISABLE_CS );
        
        BYTE bBuffer[4];
        if ( 0 == AT25DN011_readid( dev, bBuffer, sizeof( bBuffer )) ) 
        {
            if ( 0 == memcmp( bBuffer, AT25DN011_JDECID, sizeof( AT25DN011_JDECID )) )
            {
                dwRet = 0;
            }
        }     
    }
    
    return dwRet;
}

/*****************************************************************************/

uint32_t    AT25DN011_deinit( TSwSpi* dev )
{
    (void)dev;
    return 0;
}

/*****************************************************************************/

uint32_t AT25DN011_write(TSwSpi* dev, const DWORD dwAddr, BYTE* pbBuffer, const size_t szSize)
{
    uint32_t dwRet = ENXIO;
    
    if ( ( NULL != dev ) && ( NULL != pbBuffer ) )
    {
        BYTE bBuffer[AT25DN011_PAGE_SIZE];
        uint16_t iPageCount;
        const uint16_t iTotalPages = (( ( dwAddr % AT25DN011_PAGE_SIZE ) + szSize ) / AT25DN011_PAGE_SIZE ) + 1;
        uint32_t dwRelAddr = 0;
        
        for ( iPageCount = 0; iPageCount < iTotalPages; iPageCount++ )
        {
            uint32_t dwBytestoWrite;
            uint32_t dwPageAddr =   ((dwAddr + dwRelAddr) / AT25DN011_PAGE_SIZE) * AT25DN011_PAGE_SIZE;
            uint32_t dwRelMemAddr = (dwAddr + dwRelAddr) % AT25DN011_PAGE_SIZE; 
            
            dwBytestoWrite = AT25DN011_PAGE_SIZE;
            
            if ( iPageCount == 0 )
            {
                if ( iTotalPages > 1 )
                {
                    dwBytestoWrite = AT25DN011_PAGE_SIZE - (dwRelMemAddr);
                }
                else
                {
                    dwBytestoWrite = szSize;
                }
            }
            else
            {
                if ( iPageCount == (iTotalPages - 1) )
                {
                    dwBytestoWrite = szSize - dwRelAddr;
                }                
            }
            
            Flash_Busy();
            
            // read a buffer from external flash memory
            AT25DN011_read( dev, dwPageAddr, (void*)bBuffer, AT25DN011_PAGE_SIZE );
            
            // copy the dwBytestoWrite bytes from pbBuffer to bBuffer
            memcpy( &bBuffer[dwRelMemAddr],  pbBuffer + dwRelAddr, dwBytestoWrite );
            
            // issue a page erase command 
            AT25DN011_PageErase( dev, dwPageAddr );            

            const BYTE bCmdBuffer[] = 
            {        
                [0] = (BYTE)( AT25DN011_Page_Program    ),
                [1] = (BYTE)( dwPageAddr >> 16          ),
                [2] = (BYTE)( dwPageAddr >>  8          ),
                [3] = (BYTE)( dwPageAddr >>  0          ),
            };

            // send the write enable command 
            AT25DN011_wren( dev );  
            Flash_Busy();
            
            dev->EnableCS( dev );
            dev->Send( dev, bCmdBuffer[0] );
            dev->Send( dev, bCmdBuffer[1] );
            dev->Send( dev, bCmdBuffer[2] );
            dev->Send( dev, bCmdBuffer[3] );
            
            for ( volatile uint16_t uiByteCounter = 0; uiByteCounter < AT25DN011_PAGE_SIZE; uiByteCounter++ )
            {
                dev->Send( dev, bBuffer[uiByteCounter] );
            }            
            
            dev->DisableCS( dev );
            
            Flash_Busy();
            
            // send the write disable command 
            AT25DN011_wrdi( dev );              

            dwRelAddr += dwBytestoWrite;
        }
        
        if ( iTotalPages == iPageCount )
        {
            Flash_Busy();  
            dwRet = 0;
        }
    }
    return dwRet;
}

/*****************************************************************************/

uint32_t AT25DN011_read(TSwSpi* dev, const DWORD dwAddr, BYTE* pbBuffer, size_t szSize)
{
    size_t szCounterData = 0;
    if ( dev )
    {
        if ( pbBuffer )
        {
            const DWORD addr = (dwAddr & 0x00FFFFFF);
            
            const BYTE bCmdBuffer[] = 
            {
              [0] = (BYTE)(AT25DN011_Read_Array),
              [1] = (BYTE)( addr >> 16 ),
              [2] = (BYTE)( addr >> 8 ),
              [3] = (BYTE)( addr ),
              [4] = (BYTE)( 0x55 ),
            };
            
            dev->EnableCS(dev);
            
            dev->Send( dev, bCmdBuffer[0] );
            dev->Send( dev, bCmdBuffer[1] );
            dev->Send( dev, bCmdBuffer[2] );
            dev->Send( dev, bCmdBuffer[3] );
            
            if ( AT25DN011_Read_Array == bCmdBuffer[0] )
            {
                dev->Send( dev, bCmdBuffer[4] );
            }
            
            for ( volatile unsigned short int counter = 0; counter < szSize; counter++ )
            {
                dev->Receive( dev, pbBuffer + counter );                 
            }
            
            szCounterData = szSize;
            
            dev->DisableCS(dev);
        }
    }
    return szCounterData;
}

/*****************************************************************************/

uint32_t    AT25DN011_readid( TSwSpi* dev, BYTE* pbBuffer, const size_t szSize )
{
    uint32_t dwRet = ENXIO;
    if ( dev )
    {
        if ( pbBuffer )
        {
            if ( szSize >= 4 )
            {
                uint8_t bBuffer[4];
                const uint8_t bCmdBuffer[] = 
                {
                    [0] = (BYTE)(AT25DN011_Read_ManufID),
                };
                
                dev->EnableCS( dev );
                dev->Send( dev, bCmdBuffer[0] );
                dev->Receive( dev, &bBuffer[0] );
                dev->Receive( dev, &bBuffer[1] );
                dev->Receive( dev, &bBuffer[2] );
                dev->Receive( dev, &bBuffer[3] );
                dev->DisableCS( dev );

                memcpy( pbBuffer, &bBuffer[0], 4 );
                
                dwRet = 0;
            }
        }
    }
    
    return dwRet;
}

/*****************************************************************************/

uint32_t    AT25DN011_wren( TSwSpi* dev )
{
    DWORD dwRet = ENXIO;
    
    if ( dev )
    {
        const uint8_t bCmdBuffer[] = 
        {
            [0] = (BYTE)(AT25DN011_Write_Enable),
        };        
        
        dev->EnableCS( dev );
        dev->Send( dev, bCmdBuffer[0] );        
        dev->DisableCS( dev );
    }
    return dwRet;
}

/*****************************************************************************/

uint32_t    AT25DN011_wrdi( TSwSpi* dev )
{
    DWORD dwRet = ENXIO;
    
    if ( dev )
    {
        const uint8_t bCmdBuffer[] = 
        {
            [0] = (BYTE)(AT25DN011_Write_Disable),
        };           
        
        dev->EnableCS( dev );
        dev->Send( dev, bCmdBuffer[0] );        
        dev->DisableCS( dev );
    }
    return dwRet;    
}

/*****************************************************************************/

uint32_t    AT25DN011_rdst( TSwSpi* dev, void* reg )
{
    DWORD dwRet = ENXIO;
    
    volatile Flash_AT25DN011_Status_Reg* xStatusRegAT25DN011 = (Flash_AT25DN011_Status_Reg*)reg;
    
    if ( dev )
    {
        if ( reg )
        {
            uint8_t cBuffer[2];
            const uint8_t bCmdBuffer[] = 
            {
                [0] = AT25DN011_Read_Status_Register,
            };
            
            dev->EnableCS( dev );
            dev->Send( dev, bCmdBuffer[0] );    
            dev->Receive( dev, &cBuffer[0] );
            dev->Receive( dev, &cBuffer[1] );
            dev->DisableCS( dev );
        
            xStatusRegAT25DN011->reg1 = cBuffer[0];
            xStatusRegAT25DN011->reg2 = cBuffer[1];
        }
    }
    return dwRet;    
}

/*****************************************************************************/

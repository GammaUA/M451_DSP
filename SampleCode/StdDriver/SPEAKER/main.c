
/******************************************************************************
 * @file     main.c
 * @version  V0.1
 * $Revision: 1 $
 * $Date: 12/01/08 $
 * @brief    Demonstrate speech recodring - coding - transitting - receivind - decoding - playing
 * @note     Author: Sergey Gayevsky, 'Gamma UA', info@nuvoton.com.ua
 * Licence: LGPL 3.0 <http://www.gnu.org/licenses/lgpl.html>
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "arm_math.h"

//---------------------AMR-------------------
#include "interf_enc.h"
#include "interf_dec.h"
//-------------------------------------------

//ADC settings
#define ADC_MOD 0  //EADC module used for recording
#define ADC_CH 1   //PB1, pin45 EADC channel used for recording

//DMA settingth
#define ADC_PDMA 2 //PDMA channel used for recording
#define DAC_PDMA 0 //PDMA channel used for playing
#define TXD_PDMA 1 //PDMA channel used for data UART0 TX 
#define RXD_PDMA 4 //pdma channel used for data UART0 RX

//TMR settingth
#define TMR8K 1501       //12 MHz timer divider for 8000KHz sampling rate

//other settings
#define FRAMELENGTH  160 //number of PCM sampling in AMR frame
#define PKTLENGTH 12     //number of bytes in UART data packet for AMR mode 0
#define RINGBUFLEN 32    //volume of ring buffer (maximum number of received packets)
#define GUARD 4          //critical ring buffer fill (for jitter compensation)
#define KRATE 40         //koefficient of playing rate variation (ratio of delta of 'timer compare value' to delta 'buffer fill' from optimal) 
#define KFILL 16         //koefficient of measured jitter adaptation

//------------------------------------------AMR---------------------------
int *amrenstate=0; //AMR encode state
int *amrdestate=0; //AMR decode state
char amrmode=0; //4750 bps mode
char amrcbr=0; //vbr active
const char amr_block_size[8]={12, 13, 15, 17, 19, 20, 26, 31 }; //AMR packet length for mode
//-------------------------------------------------------------------------

//general variables
volatile int a=0; //for restore UART
int i;
short j, k, n;

uint16_t *pcm_pos; //pointer to used pcm frame
uint8_t *pkt_pos;  //pointer to used data packet

//ring buffer
uint8_t ring_buf[RINGBUFLEN][PKTLENGTH]; //ring buffer
uint8_t ring_in_ptr=0; //pointer to frame will be received
uint8_t ring_out_ptr=0; //pointer to frame will be transmitted
short ring_max_fill=GUARD*KFILL; //statistic minimal number of frames in ring buffer during work
short ring_min_fill=GUARD*KFILL; //statistic maximal number of frames in ring buffer during work
short delta_rate=0; //playing rate adjusting (0 is nominal rate 8000 Hz)
uint8_t record_pause=0; //flag of recording speech pause
uint8_t play_pause=0; //flag of playing speech pause 

//doube buffers
uint16_t pcm_in[2*FRAMELENGTH];   //PCM recording double buffer
uint16_t pcm_out[2*FRAMELENGTH];  //PCM playing double buffer
uint8_t pkt_in[2*PKTLENGTH]; //UART0 packet receiving double buffer
uint8_t pkt_out[PKTLENGTH];       //UART0 packet transmitting single buffer

//recording & playing flags
volatile uint8_t pcm_in_flag=0;  //flag of frame was recorded (avaliable in buf 0 or 1)
volatile uint8_t pcm_in_buf=0;   //number of buffer where pkt was recorded  (avaliable in buf 0 or 1)

volatile uint8_t pcm_out_flag=0; //flag of frame was played (in buf 1 or 2)
volatile uint8_t pcm_out_buf=0;  //flag of empty playing buffer  (we can write new to buf 0 or 1)

volatile uint8_t pkt_in_flag=0; //flag of packet was received
volatile uint8_t pkt_in_buf=0;  //number of buffer recorded now
volatile uint8_t pkt_out_buf=0; //number of buffer will be decoded
volatile uint8_t pkt_out_flag=0; //flag of pkt is transmitted now
volatile uint8_t pkt_sync=1;  //packet boundary aligning flag
volatile uint8_t mute=0; //mute flag

//Function prototype declaration
void SYS_Init(void);   //first initialization
void amr_ini(int dtx); //amr codec initialization
void amr_fin(void);    //amr finalization (not used here)
void ReloadPDMA_EADC(uint8_t dbuf); //start recording next frame 
void ReloadPDMA_DAC(uint8_t dbuf);  //start playing next frame
void SendPDMA_TXD(uint8_t* buf);    //start transmitting next data packet
void ReloadPDMA_RXD(uint8_t qbuf);  //start receiving next data packet


//main procedure
int32_t main(void)
{  
	 
	//init clocks and pins
    SYS_Init();  
    
    //clears PCM buffers
	   for(i=0;i<2*FRAMELENGTH;i++)
		 {
			 pcm_in[i]=0;
			 pcm_out[i]=0;			 
		 }
	 
		//Open UARTs 
    UART_Open(UART0, 115200); //data port

	  //Init timer for recording rate 8K fixed
    TIMER_SET_CMP_VALUE(TIMER0, TMR8K);
    TIMER0->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_TRGEADC_Msk;

    //Init timer for playing rate 8K adjustable
    TIMER_SET_CMP_VALUE(TIMER1, TMR8K);
    TIMER1->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_TRGDAC_Msk;
 	 
	 //configure ADC: module=0, ch=1, flag on end,  trigget=TMR0, DMA=160 samples
	 /* Set the ADC internal sampling time, input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);
    EADC_SetInternalSampleTime(EADC, 1);
		EADC_SET_DMOF(EADC, EADC_CTL_DMOF_TWOS_COMPLEMENT);
    /* Configure the sample module 0 for analog input channel 1 and enable Timer0 trigger source */
    EADC_ConfigSampleModule(EADC, ADC_MOD, EADC_TIMER0_TRIGGER, ADC_CH);
    EADC_SetExtendSampleTime(EADC, ADC_MOD, 168);
	  EADC_ENABLE_PDMA(EADC);
	
//-------------------------------------------------------------------------------	
	//configure DAC: trigger=TMR1, DMA=160 samples
    DAC_Open(DAC, 0, DAC_TIMER1_TRIGGER);
    DAC_SetDelayTime(DAC, 8);
		DAC_ENABLE_LEFT_ALIGN(DAC);
    DAC_CLR_INT_FLAG(DAC, 0);
		DAC_ENABLE_PDMA(DAC);
				
//==============================configure DMA====================================
	  SYS_ResetModule(PDMA_RST); //reset PDMA module
    //open DMA channels: ADC, DAC, UART0RX, UART0TX
    PDMA_Open((1<<ADC_PDMA)|(1<<DAC_PDMA)|(1<<TXD_PDMA)|(1<<RXD_PDMA));
	
//--------------PDMA for EADC----------------------
    PDMA_SetTransferCnt(ADC_PDMA, PDMA_WIDTH_16, FRAMELENGTH);
    PDMA_SetTransferAddr(ADC_PDMA, (uint32_t)&EADC->DAT[ADC_MOD], PDMA_SAR_FIX, (uint32_t)pcm_in, PDMA_DAR_INC);
    PDMA_SetTransferMode(ADC_PDMA, PDMA_ADC_RX, FALSE, 0);
    PDMA_SetBurstType(ADC_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_4);
		PDMA_EnableInt(ADC_PDMA, PDMA_INT_TRANS_DONE);
   
//--------------PDMA for DAC-------------------------  
    PDMA_SetTransferCnt(DAC_PDMA, PDMA_WIDTH_16, FRAMELENGTH);
    PDMA_SetTransferAddr(DAC_PDMA, (uint32_t)pcm_out, PDMA_SAR_INC, (uint32_t)&DAC->DAT, PDMA_DAR_FIX);
    PDMA_SetTransferMode(DAC_PDMA, PDMA_DAC_TX, FALSE, 0);
    PDMA_SetBurstType(DAC_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_128);
		PDMA_EnableInt(DAC_PDMA, PDMA_INT_TRANS_DONE);
	
//--------------PDMA for UART0 TX-------------------------  
		PDMA_SetTransferCnt(TXD_PDMA, PDMA_WIDTH_8, PKTLENGTH); //40 bytes
    PDMA_SetTransferAddr(TXD_PDMA, (uint32_t)pkt_out, PDMA_SAR_INC, (uint32_t)&UART0->DAT, PDMA_DAR_FIX);   
    PDMA_SetTransferMode(TXD_PDMA, PDMA_UART0_TX, FALSE, 0);
    PDMA_SetBurstType(TXD_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_128);
    PDMA->DSCT[TXD_PDMA].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk; 
		PDMA_EnableInt(TXD_PDMA, PDMA_INT_TRANS_DONE);	
		
//--------------PDMA for UART0 RX-------------------------  
		PDMA_SetTransferCnt(RXD_PDMA, PDMA_WIDTH_8, PKTLENGTH); //40 bytes
    PDMA_SetTransferAddr(RXD_PDMA, (uint32_t)&UART0->DAT, PDMA_SAR_FIX, (uint32_t)pkt_in, PDMA_DAR_INC);    
    PDMA_SetTransferMode(RXD_PDMA, PDMA_UART0_RX, FALSE, 0);
    PDMA_SetBurstType(RXD_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_128);
		PDMA_EnableInt(RXD_PDMA, PDMA_INT_TRANS_DONE);         
    UART0->INTEN |= UART_INTEN_RXPDMAEN_Msk; 
//--------------------------------------------------------------

		NVIC_EnableIRQ(PDMA_IRQn);  //enable DMA interupt		
//==============================================================
		
		//Initialize AMR codec engine
    amr_ini(1); //dtx0/1
		
		TIMER_Start(TIMER0); //start recording timer
		TIMER_Start(TIMER1); //start playing timer
			
    //main infinite loop
    while(1) 
		{				
//------------------------------------------------------------------------------			
//force sync mode on UART0 RX error 
			if(UART_GET_INT_FLAG(UART0, UART_INTSTS_HWRLSINT_Msk | UART_INTSTS_HWRLSIF_Msk))
			{
				UART0->INTEN &= ~UART_INTEN_RXPDMAEN_Msk; //disable DMA mode
				a=UART0->DAT; //clear UART RX data flag
				UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk); //enable interupts afrer each received bytes 	
			}
//------------------------------------------------------------------------------			
			
//recording and sending
			if(pcm_in_flag) //frame was recorded and avaliable in pcm_in_buf
			{	
				pcm_in_flag=0; //clear recording flag
				if(pcm_in_buf)  pcm_pos=pcm_in+FRAMELENGTH; else pcm_pos=pcm_in; //position of recently recorded frame
				//input is signed and right aligned  (12-bit signed format) !!! 
				//multipled by 16 in sc_enc.c, Pre_Process(), line 10871
				i=AMR_encode( amrenstate, amrmode, (int16_t*)pcm_pos, pkt_out, amrcbr );
				if(!(pkt_out[0]&1)) //check for VAD flag 
				{	 //cleared: frame is speech
				 if(record_pause) pkt_out[0]|=1; //check for previous speech pause, set flag
				 record_pause=0; //clear previous pause flag	
				 pkt_out_flag=1; //set output flag
				 SendPDMA_TXD(pkt_out); //set packet over uart
				}
				else record_pause=1; //or set speech pause recording flag if current frame is not speech
			} //end of recording
//------------------------------------------------------------------------------

//receiving				
			if(pkt_in_flag) //new packet was received and ready in double buffer now
			{
			 pkt_in_flag=0; //clear receiving flag	
			 pkt_pos = pkt_in + PKTLENGTH - pkt_in_buf*PKTLENGTH; //pointer to pkt was received	
			 
       //obtain number of packets in ring buffer and mesure minimal and maximal fill levels				
			 n=ring_in_ptr-ring_out_ptr; //number of packets in ring buffer	
			 if(n<0) n+=RINGBUFLEN; //ring				
			 j=n*KFILL; //koefficient for slowly update levels
			 
			
			 //after speech pause:
       if(pkt_pos[0]&1) //check silency flag
			 { //insert some silency
				j=(ring_max_fill-ring_min_fill)/(2*KFILL);  //actual jitter	(frames)
        if(j<0) j=0;				 
				k=GUARD+j-n; //number of silency frames insert for optimal fill 
				if(k>RINGBUFLEN/2) k=RINGBUFLEN/2;
				if(k<1) k=1; //at least one frame of silency must be inserted				 
        while(k--)
				{ //insert silency
         ring_buf[ring_in_ptr++][0]=1; //set silency flag in inserted packet
				 if(ring_in_ptr==RINGBUFLEN) ring_in_ptr=0; //ring	
				 n++; //count number of frames in ring buffer now
				}
				
        ring_min_fill=(n-j)*KFILL; //new expected minimal buffer level
				if(ring_min_fill<0) ring_min_fill=0; //must be positive
				ring_max_fill=ring_min_fill; //set zero jitter after eack pause
			
			 }
			 else //process subsequent speech data
			 {
				if(j>ring_max_fill) ring_max_fill=j; else ring_max_fill--; //update maximal buffer level
			  if(j<ring_min_fill) ring_min_fill=j; else ring_min_fill++;	//update minimal buffer level
		
        if(ring_min_fill<0)	ring_min_fill=0;   //fill always positive or null
        if(ring_min_fill>(RINGBUFLEN*KFILL/2)) ring_min_fill=RINGBUFLEN*KFILL/2; //restrict
				if(ring_max_fill < ring_min_fill) ring_max_fill=ring_min_fill; //restrict				 
			 }
			 
			 //add received voice packet to ring buffer
			 pkt_pos[0]&=0xFE; //clear silency flag
			 memcpy(ring_buf[ring_in_ptr], pkt_pos, PKTLENGTH); //copy data to buffer
			 ring_in_ptr++; //move in pointer
			 if(ring_in_ptr==RINGBUFLEN) ring_in_ptr=0; //ring	

			 //compute optimal delta for playing rate is proportional
       //to difference between measured minimal buffer fill and specified critical level			 
 			 delta_rate=KRATE*( (GUARD*KFILL) - ring_min_fill)/KFILL;
       if(delta_rate>300) delta_rate=300;  //restrict
       if(delta_rate<-300) delta_rate=-300;	//restrict		 
			 
			}
			
			
			//------------------------------------------------------------------------------			
//playing		
			if(pcm_out_flag) //frame was played and pcm_out_buf is empty now
			{
				pcm_out_flag=0; //clear playing flag
				//set pointer to the frame will be played
        if(pcm_out_buf) pcm_pos=pcm_out+FRAMELENGTH; else pcm_pos=pcm_out; 
				
				//check ring buffer is empty
				if(ring_in_ptr==ring_out_ptr) //buffer empty: underrun
				{
					memset(pcm_pos, 0, 320);//play silency if buffer insufficicent (underrun)
					delta_rate=0; //set nominal playing rate
					if(!play_pause) //check for playing pause started just now
					{  //check UART RX packet boundary align
						k=(PDMA->DSCT[RXD_PDMA].CTL)>>16; //get number of DMA transfers left: must be PKTLENGTH-1
            if(k!=(PKTLENGTH-1)) ReloadPDMA_RXD(pkt_in_buf); //restart UART receiver	in DMA mode
						play_pause=1; //set playing pause flag
					}
				}
				else if(ring_buf[ring_out_ptr][0]&1) //silency in buf
				{
				 memset(pcm_pos, 0, 320); //play silency
         ring_out_ptr++; //move ring buffer output pointer
				 if(ring_out_ptr==RINGBUFLEN) ring_out_ptr=0; //ring	
				}
				else //decode speech frame
				{
				 AMR_decode(amrdestate, amrmode, ring_buf[ring_out_ptr], (int16_t*)pcm_pos, 0); //bfi=0	
				 ring_out_ptr++; //move ring buffer output pointer
				 if(ring_out_ptr==RINGBUFLEN) ring_out_ptr=0; //ring
         play_pause=0; //clear playing pause flag					
				}
					
				for(i=0;i<FRAMELENGTH;i++) pcm_pos[i]^=0x8000; //convert signed short PCM samples to unsigned format for DAC
			} //end of playing
			
		} //end of loop
		
} //end of main



//---------------------------------------------------------------------------------------------------------
// PDMA interrupt handler                                                                                  
//---------------------------------------------------------------------------------------------------------
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(); //read DMA IRQ flags
	 
    if(status & PDMA_INTSTS_ABTIF_Msk)    // DMA abort event 
    {
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIFn_Msk); //clear DMA Abort flag
    } //end of DMA Abort event handler
    
		if(status & PDMA_INTSTS_TDIF_Msk) // DMA done event 
    {
       //ADC DMA done (frame was recorded) 
			 if(PDMA_GET_TD_STS() & (1<<ADC_PDMA))  
				{
					ReloadPDMA_EADC(pcm_in_buf);  //restart ADC DMA for recording next frame
					pcm_in_buf^=1;  //swap buffer
					pcm_in_flag = 1; //set "frame ready" flag
					PDMA_CLR_TD_FLAG( (1<<ADC_PDMA) ); //clear DMA interupt flag
				} //end of IRQ DMA ADC
				
				//DAC DMA done (frame was played)
				if(PDMA_GET_TD_STS() & (1<<DAC_PDMA)) 
				{
          ReloadPDMA_DAC(pcm_out_buf); //restart DAC DMA for playing next frame
					pcm_out_buf^=1; //swap buffer
					pcm_out_flag = 1; //set "playbuffer is empty" flag
					PDMA_CLR_TD_FLAG( (1<<DAC_PDMA) ); //clear DMA interupt flag
				}	//end of IRQ DMA DAC
        
				//UART0 TX DMA done
        if (PDMA_GET_TD_STS() & (1<<TXD_PDMA)) 
        {
          UART0->INTEN &= (~UART_INTEN_TXPDMAEN_Msk); //disable UART transmitter
					pkt_out_flag=0;  //clear flag "data in transmitting"
					PDMA_CLR_TD_FLAG( (1<<TXD_PDMA) ); //clear DMA interupt flag				
        } //end of IRQ DMA UART0 TX
				
				//UART0 RX DMA done       
        if (PDMA_GET_TD_STS() & (1<<RXD_PDMA)) //uart0 RX DONE
        {
  				 pkt_in_flag=1; //set flag "received data is ready"
					 pkt_in_buf++; //set next receiving buffer part for receiving
					 pkt_in_buf&=1; //receiving buffer is a circullar buffer
					 ReloadPDMA_RXD(pkt_in_buf); //restart UART receiver
					 PDMA_CLR_TD_FLAG( (1<<RXD_PDMA) );		//clear DMA interupt flag
        } //end of IRQ DMA UART0 RX
						
    } //end of DMA done event handler
    
} //end of DMA IRQn_Type handler



//-----------------------------------------------------------------------------------
//IRQ handle UART0 data & errors
//-----------------------------------------------------------------------------------
void UART0_IRQHandler(void)
{	 //resume DMA UART RX after one byte was succesfully received
  UART_DisableInt(UART0, UART_INTEN_RDAIEN_Msk); //disable interupts on each byte received
	UART0->INTEN |= UART_INTEN_RXPDMAEN_Msk;	//enable DMA RX mode
  ReloadPDMA_RXD(pkt_in_buf); //restart UART receiver	in DMA mode	   
	UART_ClearIntFlag(UART0 , UART_INTSTS_BUFERRINT_Msk | UART_INTSTS_RLSINT_Msk); //clears flags
}


//-----------------------------------------------------------------------------------
//Systen initialization
//-----------------------------------------------------------------------------------

//system initialization: clock setup and perifery settings
//NOTE: some perifery unused in this project was enabled for next
//disable unused periferi modules for saving power
void SYS_Init(void)
{		
		
   //-------------------------Init System Clock---------------------------------------
  
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable clock source */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock source ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_HXTSTB_Msk);

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* Set PLL frequency */
    CLK->PLLCTL = (CLK->PLLCTL & ~(0x000FFFFFul)) | 0x0000C02Eul;

    /* Waiting for PLL ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* If the defines do not exist in your project, please refer to the related clk.h in the clk_h folder appended to the tool package. */
    /* Set HCLK clock */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK-related clock */
    CLK->CLKSEL0 = CLK->CLKSEL0 & ~CLK_CLKSEL0_PCLK0SEL_Msk;
    CLK->CLKSEL0 = CLK->CLKSEL0 | CLK_CLKSEL0_PCLK1SEL_Msk;

    /* Enable IP clock */
    CLK_EnableModuleClock(ACMP01_MODULE);
    CLK_EnableModuleClock(CRC_MODULE);
    CLK_EnableModuleClock(DAC_MODULE);
    CLK_EnableModuleClock(EADC_MODULE);
    CLK_EnableModuleClock(I2C1_MODULE);
    CLK_EnableModuleClock(ISP_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);
		//CLK_EnableModuleClock(PWM1_MODULE);
		CLK_EnableModuleClock(PWM0_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);
    CLK_EnableModuleClock(SPI1_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_HCLK, 0);

    /* Set IP clock */
    CLK_SetModuleClock(EADC_MODULE, MODULE_NoMsk, CLK_CLKDIV0_EADC(8));
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, MODULE_NoMsk);
		CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_HXT, MODULE_NoMsk);
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PLL, MODULE_NoMsk);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, MODULE_NoMsk);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HXT, MODULE_NoMsk);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HXT, MODULE_NoMsk);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HXT, MODULE_NoMsk);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

   
    SystemCoreClockUpdate(); //update system clocks

//-------------------------Init Pins---------------------------------------
    	/********************
MCU: M451LG6AE(LQFP48)
Module Configuration:
PB.5(Pin:1)
PB.6(Pin:2)
ACMP0_N(Pin:48)
ACMP0_O(Pin:14)
ACMP0_P0(Pin:3)
PD.0(Pin:5)
PD.1(Pin:7)
PD.2(Pin:8)
PD.3(Pin:9)
XT1_IN(Pin:16)
XT1_OUT(Pin:15)
PC.0(Pin:19)
PC.1(Pin:20)
PC.2(Pin:21)
PC.3(Pin:22)
PC.4(Pin:23)
PC.5(Pin:25)
PC.6(Pin:26)
PC.7(Pin:27)
PE.0(Pin:24)
ICE_CLK(Pin:28)
ICE_DAT(Pin:29)
I2C1_SCL(Pin:30)
I2C1_SDA(Pin:31)
SPI1_CLK(Pin:35)
SPI1_MISO(Pin:32)
SPI1_MOSI(Pin:33)
SPI1_SS(Pin:34)
UART0_RXD(Pin:37)
UART0_TXD(Pin:38)
UART1_RXD(Pin:39)
UART1_TXD(Pin:40)
DAC(Pin:44)
ADC_CH1(Pin:45)
SPI0_CLK(Pin:46)
SPI0_MISO0(Pin:47)
********************/
	

    //If the defines do not exist in your project, please refer to the related sys.h in the sys_h folder appended to the tool package.
    //SYS->GPA_MFPH = 0x00000000;
		SYS->GPA_MFPL = SYS_GPA_MFPL_PA1MFP_PWM1_CH4; //pwm output
    SYS->GPA_MFPL = SYS_GPA_MFPL_PA3MFP_UART0_RXD | SYS_GPA_MFPL_PA2MFP_UART0_TXD | SYS_GPA_MFPL_PA1MFP_UART1_RXD | SYS_GPA_MFPL_PA0MFP_UART1_TXD;
    SYS->GPB_MFPH = 0x00000000;
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB7MFP_ACMP0_P0 | SYS_GPB_MFPL_PB5MFP_EADC_CH13| SYS_GPB_MFPL_PB4MFP_ACMP0_N | SYS_GPB_MFPL_PB1MFP_EADC_CH1 | SYS_GPB_MFPL_PB0MFP_DAC;
    SYS->GPC_MFPH = 0x00000000;
    SYS->GPC_MFPL = 0x00000000;
    SYS->GPD_MFPH = 0x00000000;
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD7MFP_ACMP0_O;
    
		SYS->GPE_MFPH = SYS_GPE_MFPH_PE13MFP_SPI1_CLK | SYS_GPE_MFPH_PE12MFP_SPI1_SS | SYS_GPE_MFPH_PE11MFP_SPI1_MOSI | SYS_GPE_MFPH_PE10MFP_SPI1_MISO | SYS_GPE_MFPH_PE9MFP_I2C1_SDA | SYS_GPE_MFPH_PE8MFP_I2C1_SCL;
    SYS->GPE_MFPL = SYS_GPE_MFPL_PE0MFP_PWM0_CH0;
		
    SYS->GPF_MFPL = SYS_GPF_MFPL_PF6MFP_ICE_DAT | SYS_GPF_MFPL_PF5MFP_ICE_CLK | SYS_GPF_MFPL_PF4MFP_XT1_IN | SYS_GPF_MFPL_PF3MFP_XT1_OUT;

		GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 0));  //DAC
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 1));  //EADC1
		GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));  //N 
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 5));  //CH13
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 7));  //P0		

    SYS_LockReg(); //lock system registers
    return;
}



//-----------------------------------------------------------------------------------
//Internal procedures
//-----------------------------------------------------------------------------------

//------------------------Audio interface---------------------
void ReloadPDMA_EADC(uint8_t dbuf)  //start frame recording
{
    PDMA_SetTransferCnt(ADC_PDMA, PDMA_WIDTH_16, FRAMELENGTH); //set frame length
    PDMA_SetTransferAddr(ADC_PDMA, (uint32_t)&EADC->DAT[ADC_MOD], PDMA_SAR_FIX, (uint32_t)(pcm_in+dbuf*FRAMELENGTH), PDMA_DAR_INC); //set destination
    PDMA_SetTransferMode(ADC_PDMA, PDMA_ADC_RX, FALSE, 0); //run ADC in DMA mode	  
}

void ReloadPDMA_DAC(uint8_t dbuf)  //start frame playing
{          
     TIMER_SET_CMP_VALUE(TIMER1, TMR8K+delta_rate);
	   PDMA_SetTransferCnt(0, PDMA_WIDTH_16, FRAMELENGTH); //set frame length
     PDMA_SetTransferAddr(DAC_PDMA, (uint32_t)(pcm_out+dbuf*FRAMELENGTH), PDMA_SAR_INC, (uint32_t)&DAC->DAT, PDMA_DAR_FIX); //set source 
		 PDMA_SetTransferMode(0, PDMA_DAC_TX, FALSE, 0); //run DAC in DMA	 mode
}

//----------------Data UART0 interface--------------------------
void ReloadPDMA_RXD(uint8_t qbuf)   //start receive data packet
{
		UART0->FIFO |= (1ul << 1); //clear UART0 FIFO  
		PDMA_SetTransferCnt(RXD_PDMA, PDMA_WIDTH_8, PKTLENGTH); //set fixed length of received packet
		PDMA_SetTransferAddr(RXD_PDMA, (uint32_t)&UART0->DAT, PDMA_SAR_FIX, (uint32_t)(pkt_in + qbuf*PKTLENGTH), PDMA_DAR_INC);  
    PDMA_SetTransferMode(RXD_PDMA, PDMA_UART0_RX, FALSE, 0); //run UART0 RX in DMA mode
}

void SendPDMA_TXD(uint8_t* buf) //start transmitt data packet
{
		//buf[PKTLENGTH-1]=SYNC_BYTE;
	  PDMA_SetTransferCnt(TXD_PDMA, PDMA_WIDTH_8, PKTLENGTH); //set length of transmitted data
    PDMA_SetTransferAddr(TXD_PDMA, (uint32_t)buf, PDMA_SAR_INC, (uint32_t)&UART0->DAT, PDMA_DAR_FIX);  //set sorce  
    PDMA_SetTransferMode(TXD_PDMA, PDMA_UART0_TX, FALSE, 0); //run UART0 TX in DMA mode
	  UART0->INTEN |= UART_INTEN_TXPDMAEN_Msk;
}

//---------------------------------AMR------------------------------------
//AMR initialization
void amr_ini(int dtx)
{
  amrenstate = Encoder_Interface_init(dtx); //create AMR encoder
  amrdestate = Decoder_Interface_init();    //create AMR decoder
}

//AMR finalization
void amr_fin(void)
{
 if(amrenstate) Encoder_Interface_exit(amrenstate); //free  AMR encoder
 if(amrdestate) Decoder_Interface_exit(amrdestate); //free AMR decoder
}

//---------------------------------TRAPS----------------------------------
//chack for malloc fail
void mtrap(int line) //malloc failure trap
{
 a=line; //number of code line where malloc is failure
 while(1);	
}


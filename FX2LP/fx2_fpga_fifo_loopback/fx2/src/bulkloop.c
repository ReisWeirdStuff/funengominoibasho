// =-= Slave FIFO firmware for FX2LP =-=


#include <stdio.h>

#include <fx2regs.h>
#include <fx2macros.h>
#include <serial.h>
#include <delay.h>
#include <autovector.h>
#include <lights.h>
#include <setupdat.h>
#include <eputils.h>

#define SYNCDELAY SYNCDELAY4

volatile __bit got_sud;
volatile __bit done_frm_fpga;
BYTE Configuration;
BYTE AlternateSetting;

void main(void) {

 REVCTL = 0x00;
 SYNCDELAY;

 d2off();
 got_sud = FALSE;
 done_frm_fpga = FALSE;
 Configuration = 0;
 AlternateSetting = 0;

 // renumerate
 RENUMERATE_UNCOND(); 

 // Set CPU to 48MHz, output CLKOUT
 CPUCS = 0x10; // CLKSPD[1:0]=10, for 48MHz operation
 SYNCDELAY;
 CPUCS |= 0x02; // Enable CLKOUT
 
 SETIF48MHZ();
 sio0_init(57600);

 // Configure pin flags for slave FIFO
 // FLAGA - EP6FF (EP6 Full Flag)
 PINFLAGSAB = 0x08;
 SYNCDELAY;
 // FLAGD - EP2EF (EP2 Empty Flag)
 PINFLAGSCD = 0xE0;
 SYNCDELAY;
 
 // Enable FLAGD on PA7
 PORTACFG |= 0x80;
 SYNCDELAY;

 // Configure interface for slave FIFO mode
 // IFCLKSRC=1 (internal), 48MHz=1, IFCLKOE=1, IFCLKPOL=0, ASYNC=0, GSTATE=0, IFCFG=11 (slave FIFO)
 IFCONFIG = 0xE3;
 SYNCDELAY;

 // Configure endpoints for slave FIFO operation
 // EP2: OUT, bulk, 512 bytes, 4x buffered
 EP2CFG = 0xA0;
 SYNCDELAY;
 // EP6: IN, bulk, 512 bytes, 4x buffered  
 EP6CFG = 0xE0;
 SYNCDELAY;
 // Disable unused endpoints
 EP4CFG = 0x02; // clear valid bit
 SYNCDELAY;
 EP8CFG = 0x02; // clear valid bit
 SYNCDELAY;
 EP1INCFG &= ~bmVALID;
 SYNCDELAY;
 EP1OUTCFG &= ~bmVALID;
 SYNCDELAY;

 // Reset all FIFOs
 FIFORESET = 0x80; // NAK-ALL to avoid race conditions
 SYNCDELAY;
 FIFORESET = 0x02; // reset FIFO 2
 SYNCDELAY;
 FIFORESET = 0x04; // reset FIFO 4
 SYNCDELAY;
 FIFORESET = 0x06; // reset FIFO 6
 SYNCDELAY;
 FIFORESET = 0x08; // reset FIFO 8
 SYNCDELAY;
 FIFORESET = 0x00; // deactivate NAK-ALL
 SYNCDELAY;

 // Configure EP2 for AUTOOUT mode
 // Need to see AUTOOUT=0 to AUTOOUT=1 transition to arm endpoint
 EP2FIFOCFG = 0x00; // AUTOOUT=0, WORDWIDE=0
 SYNCDELAY;
 EP2FIFOCFG = 0x11; // AUTOOUT=1, WORDWIDE=1 (16-bit)
 SYNCDELAY;

 // Configure EP6 for AUTOIN mode
 // AUTOIN=1, ZEROLENIN=1, WORDWIDE=1
 EP6FIFOCFG = 0x0D;
 SYNCDELAY;

 // Initialize Port A for JTAG enable (PA.1)
 OEA |= 0x02; // PA.1 as output
 SYNCDELAY;
 IOA |= 0x02; // output 1 on PA.1
 SYNCDELAY;

 // Initialize Port C for sync signals
 OEC |= 0x01; // PC.0 as output (SYNC signal)
 SYNCDELAY;
 IOC &= ~0x01; // output 0 on PC.0 - SYNC signal LOW
 SYNCDELAY;
 OEC &= ~0x02; // PC.1 as input (Clock changing signal from FPGA)
 SYNCDELAY;

 // Set up USB interrupts
 USE_USB_INTS(); 
 ENABLE_SUDAV();
 ENABLE_SOF();
 ENABLE_HISPEED();
 ENABLE_USBRESET();

 EA = 1; // global interrupt enable 
 printf("Slave FIFO initialized\n");

 d3off();
 
 while(TRUE) {
 
  if (got_sud) {
      printf("Handle setupdata\n");
      handle_setupdata(); 
      got_sud = FALSE;
  }

  // Slave FIFO mode - data transfer handled automatically by hardware
  // Poll IOC for FPGA sync signal (PC.1)
  if (!(IOC & 0x02)) {
      done_frm_fpga = TRUE;
  }
  
  if (done_frm_fpga && (IOC & 0x02)) {
      // Switch to ports mode temporarily if needed
      IFCONFIG = 0x03;
      SYNCDELAY;
      
      // Output 1 on PC.0 - SYNC signal HIGH
      IOC |= 0x01;
      SYNCDELAY;
      done_frm_fpga = FALSE;
  }
 }

}

// copied routines from setupdat.h

BOOL handle_get_descriptor(void) {
  return FALSE;
}

BOOL handle_vendorcommand(BYTE cmd) {
 (void)cmd; // unused
 return TRUE;
}

// this firmware only supports 0,0
BOOL handle_get_interface(BYTE ifc, BYTE* alt_ifc) { 
 printf("Get Interface\n");
 (void)ifc; // unused
 *alt_ifc = AlternateSetting; 
 return TRUE;
}

BOOL handle_set_interface(BYTE ifc, BYTE alt_ifc) { 
 printf("Set interface %d to alt: %d\n", ifc, alt_ifc);
 
 // Simply store the alternate setting - don't reset FIFOs
 // Slave FIFO mode handles this automatically
 AlternateSetting = alt_ifc;
 return TRUE;
}

// get/set configuration
BYTE handle_get_configuration(void) {
 return Configuration; 
}

BOOL handle_set_configuration(BYTE cfg) { 
 printf("Set configuration: %d\n", cfg);
 
 if (cfg == 0 || cfg == 1) {
    Configuration = cfg;
    
    // Set AUTOIN length based on speed
    if (HISPEED) {
        // High speed: 512 bytes
        EP6AUTOINLENH = 0x02;
        SYNCDELAY;
        EP6AUTOINLENL = 0x00;
        SYNCDELAY;
    } else {
        // Full speed: 64 bytes
        EP6AUTOINLENH = 0x00;
        SYNCDELAY;
        EP6AUTOINLENL = 0x40;
        SYNCDELAY;
    }
    
    return TRUE;
 }
 return FALSE;
}


// USB interrupt handlers (fx2lib autovector style)
void sudav_isr(void) __interrupt (SUDAV_ISR) {
  got_sud = TRUE;
  CLEAR_SUDAV();
}

__bit on5;
__xdata WORD sofct = 0;
void sof_isr(void) __interrupt (SOF_ISR) __using (1) {
    ++sofct;
    if (sofct == 8000) { // about 8000 SOF interrupts per second at high speed
        on5 = !on5;
        if (on5) { d5on(); } else { d5off(); }
        sofct = 0;
    }
    CLEAR_SOF();
}

void usbreset_isr(void) __interrupt (USBRESET_ISR) {
    handle_hispeed(FALSE);
    CLEAR_USBRESET();
}

void hispeed_isr(void) __interrupt (HISPEED_ISR) {
    handle_hispeed(TRUE);
    CLEAR_HISPEED();
}
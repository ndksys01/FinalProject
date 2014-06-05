#include <16f887.h>
#device  adc=10
#include <string.h>
#use delay(clock =4M)
#use rs232(baud=9600,xmit=PIN_C6,rcv=PIN_C7)

   INT temp=0;

char nhietDo[6];
float tmpNhietdo;

void init_adc(INT kenh)
{
   setup_adc (adc_clock_internal);

   IF (kenh == 0)
   {
      setup_adc_ports (sAN0|VSS_VDD);
      set_adc_channel (0);
   }

   else IF (kenh == 1)
   {
      setup_adc_ports (sAN1|VSS_VDD);
      set_adc_channel (1);
   }
}

#INT_rda

void uartInterrupt()
{
   temp = read_adc ();
   tmpNhietdo = (FLOAT) ( ((temp * 5.1 * 100) / 1023));
   nhietDo[0] = '#';
   nhietDo[1] = '5';
   nhietDo[2] = ( ( (INT) tmpNhietdo) % 100) / 10 + 48;
   nhietDo[3] = ( (INT) tmpNhietdo % 100) % 10 + 48;
   nhietDo[4] = '.';
   nhietDo[5] = 0 + 48;
}

void main()
{
   UNSIGNED char a = 0;
   init_adc (1) ;
   enable_interrupts (global); //Cho phep ngat toan cuc
   enable_interrupts (INT_rda); //Cho phep ngat UART
   WHILE (1)
   {
      
      temp = read_adc ();
      tmpNhietdo = (FLOAT) ( ((temp * 5.1 * 100) / 1023));
      nhietDo[0] = ( ( (INT) tmpNhietdo) % 100) / 10 + 48;
      nhietDo[1] = ( (INT) tmpNhietdo % 100) % 10 + 48;
      nhietDo[2] = '.';
      nhietDo[3] = 48; // (INT) (tmpNhietdo * 10) % 10  +  48;
      nhietDo[4] = '\r';
      nhietDo[5] = '\n';
      //printf (nhietDo);
      putc ('#'); delay_ms (1) ;
      putc ('1'); delay_ms (1) ;
      putc (nhietDo[0]); delay_ms (1) ;
      putc (nhietDo[1]); delay_ms (1) ;
      putc (nhietDo[2]); delay_ms (1) ;
      putc (nhietDo[3]); delay_ms (1) ;
      putc (nhietDo[4]); delay_ms (1) ;
      putc (nhietDo[5]); delay_ms (1) ;
      delay_ms (500); delay_ms (500) ;
      delay_ms (500); delay_ms (500) ;
      output_d (a);
      a = 255 - a;
   }
}


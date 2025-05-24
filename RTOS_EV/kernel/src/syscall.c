/**
 * @file 
 *
 * @brief      
 *
 * @date       
 *
 * @author     
 */

#include <unistd.h>
#include <syscall.h>
#include <uart.h>
#include <lcd_driver.h>
#include <arm.h>
#include <printk.h>
#include <i2c.h>
#define UNUSED __attribute__((unused))
extern char __heap_top;
extern char __heap_low;
char *my_heaptop=(char*)&__heap_top;
char *my_heaplow=(char*)&__heap_low;
char *my_heapcurr=(char*)&__heap_low;
void *sys_sbrk(int incr)
{
  char *prebreak=my_heapcurr;
  if((my_heapcurr+incr)>my_heaptop)
  {
    return (void*)-1;
  }
  my_heapcurr+=incr;
  return prebreak;
}

int sys_write(int file, char *buf, int len)
{
  if (file != 1) return -1;
  for (int i = 0; i < len; i++) {
    while (uart_put_byte(buf[i]));
    // uart_put_byte(buf[i]);
  }
  return len;
}

int sys_read(int file, char *ptr, int len)
{
  if(file!=0)
  {
    return -1;
  }
  int count=0;
  char c;
  while(count<len)
  {
    if(uart_get_byte(&c)==0)
    {
    if(c==4)
    {
      return count;
    }
    else if(c=='\b')
    {
      if(count>0)
      {
        count--;
        printk("\b \b");
      }
      continue;
    }
    else if(c=='\n')
    {
      ptr[count]=c;
      count++;
      printk("\n");
      return count;
    }
    else if(c=='\r')
    {
      ptr[count]=c;
      count++;
      printk("\n");
      break;
    }
    else {
      uart_put_byte(c);
      ptr[count]=c;
      count++;
    }
    
    }
  }
  return count;
}

void sys_exit(int status)
{
  printk("Exit:%d",status);
  uart_flush();
  lcd_print("Exit");
  disable_interrupts();
  while(1);
}
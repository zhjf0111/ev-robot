#include "serialport.h"

int speed_arr[15] = { B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[15] = { 115200, 38400,  19200,  9600,  4800,  2400,  1200,  300, 38400, 19200,  9600, 4800, 2400, 1200,  300, };

int openserial(char port[], int speed, int databits, int parity, int stopbits )
{
  int fd;
 // printf("open %s\n", port);
  fd = open(port, O_RDWR);
  if(fd == -1)
  {
    perror("serialport error\n");
    return -1;
  }
  else
  {
    printf("open ");
    printf("%s",ttyname(fd));
    printf(" succesfully\n");
    set_speed(fd, speed);
    set_Parity(fd, databits, parity, stopbits);
    return fd;
  }  
}

void set_speed(int fd, int speed){
  unsigned int   i; 
  int   status; 
  struct termios   Opt;
  tcgetattr(fd, &Opt); 
  for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) { 
    if  (speed == name_arr[i]) {     
      tcflush(fd, TCIOFLUSH);     
      cfsetispeed(&Opt, speed_arr[i]);  
      cfsetospeed(&Opt, speed_arr[i]);   
      status = tcsetattr(fd, TCSANOW, &Opt);  
      if  (status != 0) {        
        perror("tcsetattr fd1");  
        return;     
      }    
      tcflush(fd,TCIOFLUSH);   
      printf("%d,", name_arr[i]);
      return;
    }  
  }
}

int set_Parity(int fd,int databits,int parity, int stopbits)
{ 
  struct termios options; 
  if  ( tcgetattr( fd,&options)  !=  0) { 
    perror("SetupSerial 1");     
    return(-1);  
  }
  options.c_cflag &= ~CSIZE; 
  options.c_lflag &= ~ECHO; //disable echo 
 // options.c_iflag |= TGNCR;
  switch (databits) 
  {   
  case 7:   
    options.c_cflag |= CS7; 
    printf("7,");
    break;
  case 8:     
    options.c_cflag |= CS8;
    printf("8,");
    break;   
  default:    
    fprintf(stderr,"Unsupported data size\n"); return (-1);  
  }
  switch (parity) 
  {   
    case 'n':
    case 'N':    
      options.c_cflag &= ~PARENB;   /* Clear parity enable */
      options.c_iflag &= ~INPCK;     /* Enable parity checking */ 
      printf("N,");
      break;  
    case 'o':   
    case 'O':     
      options.c_cflag |= (PARODD | PARENB); 
      options.c_iflag |= INPCK;             /* Disnable parity checking */ 
      printf("O,");
      break;  
    case 'e':  
    case 'E':   
      options.c_cflag |= PARENB;     /* Enable parity */    
      options.c_cflag &= ~PARODD;    
      options.c_iflag |= INPCK;       /* Disnable parity checking */
      printf("E,");
      break;
    case 'S': 
    case 's':  /*as no parity*/   
        options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;break;  
      printf("S,");
    default:   
      fprintf(stderr,"Unsupported parity\n");    
      return (-1);  
    }  
  
  switch (stopbits)
  {   
    case 1:    
      options.c_cflag &= ~CSTOPB;  
      printf("1\n");
      break;  
    case 2:    
      options.c_cflag |= CSTOPB;  
      printf("2\n");
       break;
    default:    
       fprintf(stderr,"Unsupported stop bits\n");  
       return (-1); 
  } 
  /* Set input parity option */ 
  if (parity != 'n')   
    options.c_iflag |= INPCK; 
  tcflush(fd,TCIFLUSH);
  options.c_cc[VTIME] = 150; 
  options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
  if (tcsetattr(fd,TCSANOW,&options) != 0)   
  { 
    perror("SetupSerial 3");   
    return (-1);  
  } 
  return (0);  
}



/*

int main()
{
  printf("This program updates last time at %s   %s\n",__TIME__,__DATE__);
  printf("STDIO COM1\n");
  int fd;
  int cnt = 0;
  int i = 0;
  fd = open("/dev/ttyUSB1",O_RDWR);
  if(fd == -1)
  {
    perror("serialport error\n");
  }
  else
  {
    printf("open ");
    printf("%s",ttyname(fd));
    printf(" succesfully\n");
  }

  set_speed(fd,115200);
  if (set_Parity(fd,8,1,'N') == FALSE)  {
    printf("Set Parity Error\n");
    exit (0);
  }
  char buf[] = "This is a test\n";
  printf("Write %d bits\n", write(fd, &buf, sizeof(buf)));
  char buff[512]; 
  int nread;  
  printf("Prepare to read ... ...\n");
  while(1)
  {
    if((nread = read(fd, buff, 512))>1)
    {
      printf("\nLen: %d\n",nread);
      buff[nread+1] = '\0';
      for(i=0; i<nread; i++)
      	printf("%c", buff[i]);
      printf("\n");
     // printf("Receive: %s\n", buff);
      memset(buff, 0, sizeof(buff));
      printf("CNT = %d\n", cnt++);
     //write(fd,"Hello ttyUSB0\n",14);
      //printf("Write end\n");
    }

  }
  close(fd);
  return 0;
}

*/
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"

#include "radio.h"
#include "spi_.h"

#include "worked_delay.h"

//unsigned char idxIN = 0, idxOUT = 0;
//char buffer [BUF_SIZE];

unsigned char rIN = 0, rOUT = 0;
char rBuffer [BUF_SIZE];


/*

NRF 24 PINOUT:

  GRND				VCC3.3
	CE					CSN
	SCK					MOSI
	MISO				IRQ


connect STM32 PINS to NRF24 (look at SPI_.h:

  GRND -> CRND
	3.3v -> 3.3v
	A2	 -> IRQ
	A4   -> CSN
	A5	 -> SCK
  A6	 -> MISO
  A7	 -> MOSI
  C13  -> CE
	
	other pins connection:	
	C15  -> BUTTON to GRND
	A8, A9, A10 -> LEDS+
	GRND	-> LEDS-

*/



uint8_t SPIx_Transfer(uint8_t data);
void SPIx_EnableSlave(void);
void SPIx_DisableSlave(void);
 
void init_gpioc13Out_spi_GPIO_Pin_7_A();
void init_usart_button();


GPIO_InitTypeDef port;


unsigned char spi_send_recv(unsigned char data) {
  return SPIx_Transfer(data);
}

void CSN(char val)
{
	if(val==0) SPIx_EnableSlave();
	else SPIx_DisableSlave();
}

void setCE(char val)
{
	if(val==0)
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	else 
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void led_refresh(unsigned char data) {
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	GPIO_ResetBits(GPIOA, GPIO_Pin_9);
	GPIO_ResetBits(GPIOA, GPIO_Pin_10);
  if (data == 1) GPIO_SetBits(GPIOA, GPIO_Pin_8);
	if (data == 2) GPIO_SetBits(GPIOA, GPIO_Pin_9);
	if (data == 3) GPIO_SetBits(GPIOA, GPIO_Pin_10);	
}


uint8_t SPIx_Transfer(uint8_t data)
{
	// Write data to be transmitted to the SPI data register
	SPIx->DR = data;
	// Wait until transmit complete
	while (!(SPIx->SR & (SPI_I2S_FLAG_TXE)));
	// Wait until receive complete
	while (!(SPIx->SR & (SPI_I2S_FLAG_RXNE)));
	// Wait until SPI is not busy anymore
	while (SPIx->SR & (SPI_I2S_FLAG_BSY));
	// Return received data from SPI data register
	return SPIx->DR;
}

void SPIx_EnableSlave()
{
	// Set slave SS pin low
	SPI_GPIO->BRR = SPI_PIN_SS;
}

void SPIx_DisableSlave()
{
	// Set slave SS pin high
	SPI_GPIO->BSRR = SPI_PIN_SS;
}


unsigned char radio_read_buf(unsigned char cmd, unsigned char * buf, unsigned char count) ;//!
// Выполняет команду cmd, и передаёт count байт параметров из буфера buf, возвращает регистр статуса
unsigned char radio_write_buf(unsigned char cmd, unsigned char * buf, unsigned char count) ;//!
// Читает значение однобайтового регистра reg (от 0 до 31) и возвращает его
unsigned char radio_readreg(unsigned char reg) ;//!
// Записывает значение однобайтового регистра reg (от 0 до 31), возвращает регистр статуса
unsigned char radio_writereg(unsigned char reg, unsigned char val) ;//!
// Читает count байт многобайтового регистра reg (от 0 до 31) и сохраняет его в буфер buf,
// возвращает регистр статуса
//unsigned char radio_readreg_buf(unsigned char reg, unsigned char * buf, unsigned char count) ;
// Записывает count байт из буфера buf в многобайтовый регистр reg (от 0 до 31), возвращает регистр статуса
unsigned char radio_writereg_buf(unsigned char reg, unsigned char * buf, unsigned char count) ;
// Возвращает размер данных в начале FIFO очереди приёмника
unsigned char radio_read_rx_payload_width() ;//!
// Выполняет команду. Возвращает регистр статуса
unsigned char radio_cmd(unsigned char cmd) ;//!
// Возвращает 1, если на линии IRQ активный (низкий) уровень.
unsigned char radio_is_interrupt() ;
unsigned char radio_start() ;
void on_packet(unsigned char * buf, unsigned char size) ;//!!
unsigned char send_data(unsigned char * buf, unsigned char size) ;//!
void check_radio() ;//!






int main()
{
    __enable_irq ();
    init_usart_button();//
	init_gpioc13Out_spi_GPIO_Pin_7_A();   //
    delay_ms(100);
	char i=radio_start()+48;//does need for i ???
	delay_ms(2);
	setCE(1);
	
	unsigned char data = 0;
	unsigned char s_data = 0;

	while (1)
	{
		led_refresh(data);
		if (rIN != rOUT)//reseive 
		{
			data=rBuffer[rOUT++];			
			delay_ms(100);
			rOUT &= BUF_MASK;
			/*for(int i=0;i<8;i++)
			{
				if((data>>i)&1)GPIO_SetBits(GPIOC, GPIO_Pin_11);
				else GPIO_ResetBits(GPIOC, GPIO_Pin_11);
				delay_ms(100);
			}*/
		    
		}
		else if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))//send button  
		{
		  /*unsigned char buf[2];
		  buf[0] = 15; //байт данных
		  buf[1] = 18; //байт данных
		  send_data(buf, 2);*/
			s_data++;
			if (s_data > 3) 
				s_data = 1;
			send_data(&s_data, 1);
		  delay_ms(100);
		}
		
		else check_radio();//reseive
		delay_us(1000);
		
	}
}
 
void init_gpioc13Out_spi_GPIO_Pin_7_A()
{
	GPIO_InitTypeDef port;
    //Это функция из файла stm32f10x_rcc.c, включает тактирование на GPIOA
    //GPIOA сидит на шине APB2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    //Заполняем поля структуры нужными значениями
    //Первый вывод – вход для обработки нажатия кнопки – PA2
	port.GPIO_Pin = GPIO_Pin_2;
    port.GPIO_Mode = GPIO_Mode_IPU;    
    port.GPIO_Speed = GPIO_Speed_2MHz;  
    //А про эту функцию мы уже говорили   
    //Отметим только что один из параметров – указатель(!) на  
    //нашу структуру
    GPIO_Init(GPIOA, &port);    
    //Настраиваем вывод
	port.GPIO_Pin = GPIO_Pin_13;  
    port.GPIO_Mode = GPIO_Mode_Out_PP;   
    port.GPIO_Speed = GPIO_Speed_2MHz;   
    GPIO_Init(GPIOC, &port);
	
	
	// Initialization struct
	SPI_InitTypeDef SPI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// Step 1: Initialize SPI
	RCC_APB2PeriphClockCmd(SPIx_RCC, ENABLE);
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI_Init(SPIx, &SPI_InitStruct); 
	SPI_Cmd(SPIx, ENABLE);
	
	// Step 2: Initialize GPIO
	RCC_APB2PeriphClockCmd(SPI_GPIO_RCC, ENABLE);
	// GPIO pins for MOSI, MISO, and SCK
	GPIO_InitStruct.GPIO_Pin = SPI_PIN_MOSI | SPI_PIN_MISO | SPI_PIN_SCK;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStruct);
	// GPIO pin for SS
	GPIO_InitStruct.GPIO_Pin = SPI_PIN_SS;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStruct);
	
	SPIx_DisableSlave();
	
    
}

void init_usart_button()
{
    //Включаем тактирование
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
 
    //Пины PA9 и PA10 в режиме альтернативных функций –
    //Rx и Tx USART’а
	
//	//светодиод
//    GPIO_StructInit(&port);
//    port.GPIO_Mode = GPIO_Mode_Out_PP;
//    port.GPIO_Pin = GPIO_Pin_11;
//    port.GPIO_Speed = GPIO_Speed_2MHz;
//    GPIO_Init(GPIOC, &port);
 
	
	  //
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_Out_PP;
    port.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &port);
	
	
	//кнопка
    port.GPIO_Mode = GPIO_Mode_IPU;
    port.GPIO_Pin = GPIO_Pin_15;    
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &port);
}

 unsigned char radio_read_buf(unsigned char cmd, unsigned char * buf, unsigned char count) {
  unsigned char status;
  CSN(0);
  status = spi_send_recv(cmd);
  while (count--) {
    *(buf++) = spi_send_recv(0xFF);
  }
  CSN(1);
  return status;
}

// Выполняет команду cmd, и передаёт count байт параметров из буфера buf, возвращает регистр статуса
unsigned char radio_write_buf(unsigned char cmd, unsigned char * buf, unsigned char count) {
  unsigned char status;
  CSN(0);
  status = spi_send_recv(cmd);
  while (count--) {
    spi_send_recv(*(buf++));
  }
  CSN(1);
  return status;
}

// Читает значение однобайтового регистра reg (от 0 до 31) и возвращает его
unsigned char radio_readreg(unsigned char reg) {
  unsigned char answ;
  CSN(0);
  spi_send_recv((reg & 31) | R_REGISTER);
  answ = spi_send_recv(0xFF);
  CSN(1);
  return answ;
}

// Записывает значение однобайтового регистра reg (от 0 до 31), возвращает регистр статуса
unsigned char radio_writereg(unsigned char reg, unsigned char val) {
  unsigned char status;
  CSN(0);
  status = spi_send_recv((reg & 31) | W_REGISTER);
  spi_send_recv(val);
  CSN(1);
  return status;
}

// Читает count байт многобайтового регистра reg (от 0 до 31) и сохраняет его в буфер buf,
// возвращает регистр статуса
//unsigned char radio_readreg_buf(unsigned char reg, unsigned char * buf, unsigned char count) {
//  return radio_read_buf((reg & 31) | R_REGISTER, buf, count);
//}

// Записывает count байт из буфера buf в многобайтовый регистр reg (от 0 до 31), возвращает регистр статуса
unsigned char radio_writereg_buf(unsigned char reg, unsigned char * buf, unsigned char count) {
  return radio_write_buf((reg & 31) | W_REGISTER, buf, count);
}

// Возвращает размер данных в начале FIFO очереди приёмника
unsigned char radio_read_rx_payload_width() {
  unsigned char answ;
  CSN(0);
  spi_send_recv(R_RX_PL_WID);
  answ = spi_send_recv(0xFF);
  CSN(1);
  return answ;
}

// Выполняет команду. Возвращает регистр статуса
unsigned char radio_cmd(unsigned char cmd) {
  unsigned char status;
  CSN(0);
  status = spi_send_recv(cmd);
  CSN(1);
  return status;
}

// Возвращает 1, если на линии IRQ активный (низкий) уровень.
unsigned char radio_is_interrupt() {
  return !GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);  
}



unsigned char radio_start() {

  unsigned char remote_addr[] = {0xAD, 0xAD, 0xAD, 0xAB, 0xAD}; // Адрес удалённой стороны
  unsigned char cnt;
  setCE(0);

  for (cnt = 100;;) {
    radio_writereg(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PRIM_RX)); // Выключение питания
    if (radio_readreg(CONFIG) == ((1 << EN_CRC) | (1 << CRCO) | (1 << PRIM_RX)))
      break;
    // Если прочитано не то что записано, то значит либо радио-чип ещё инициализируется, либо не работает.
    if (!cnt--)return 0; // Если после 100 попыток не удалось записать что нужно, то выходим с ошибкой
    delay_ms(1);
  }

  radio_writereg(EN_AA, (1 << ENAA_P1)); // включение автоподтверждения только по каналу 1
  radio_writereg(EN_RXADDR, (1 << ERX_P0) | (1 << ERX_P1)); // включение каналов 0 и 1
  radio_writereg(SETUP_AW, SETUP_AW_3BYTES_ADDRESS); // выбор длины адреса 5 байт

  radio_writereg(SETUP_RETR, (4 << ARD) | (10 << ARC) );
  radio_writereg(RF_CH, 0); // Выбор частотного канала
  radio_writereg(RF_SETUP, (1 << RF_DR_HIGH) | (3 << RF_PWR)); //

  radio_writereg_buf(RX_ADDR_P0, &remote_addr[0], 5); // Подтверждения приходят на канал 0
  radio_writereg_buf(TX_ADDR, &remote_addr[0], 5);

  radio_writereg_buf(RX_ADDR_P1, &remote_addr[0], 5);

  radio_writereg(RX_PW_P0, 0);
  radio_writereg(RX_PW_P1, 32);
  radio_writereg(DYNPD, (1 << DPL_P0) | (1 << DPL_P1)); // включение произвольной длины для каналов 0 и 1
  radio_writereg(FEATURE, 0x04); // разрешение произвольной длины пакета данных

  radio_writereg(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX)); // Включение питания

  return (radio_readreg(CONFIG) == ((1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX))) ? 1 : 0;
}



void on_packet(unsigned char * buf, unsigned char size) {
  char i;
  for (i = 0; i < size; i++)
  {
    rBuffer[rIN++] = buf[i];
    rIN &= BUF_MASK;
  }
  delay_us(100);
}

unsigned char send_data(unsigned char * buf, unsigned char size) {
  unsigned char conf;
  unsigned char status;
  setCE(0);
  conf = radio_readreg(CONFIG);
  if (!(conf & (1 << PWR_UP))) // Если питание по какой-то причине отключено, возвращаемся с ошибкой
    return 0;
  status = radio_writereg(CONFIG, conf & ~(1 << PRIM_RX)); // Сбрасываем бит PRIM_RX
  if (status & (1 << TX_FULL_STATUS))  // Если очередь передатчика заполнена, возвращаемся с ошибкой
    return 0;
  radio_write_buf(W_TX_PAYLOAD, buf, size); // Запись данных на отправку
  setCE(1); // Импульс на линии CE приведёт к началу передачи
  delay_us(100); // Нужно минимум 10мкс, возьмём с запасом
  setCE(0);
  return 1;
}

void check_radio() {
  unsigned char status;
  unsigned char conf;
  unsigned char protect = 32; // В очереди FIFO не должно быть более 3 пакетов. Если больше, значит что-то не так
  if (!radio_is_interrupt()) // Если прерывания нет, то не задерживаемся
    return;
  status = radio_cmd(NOP);
  radio_writereg(STATUS, status); // Просто запишем регистр обратно, тем самым сбросив биты прерываний

  if (status & ((1 << TX_DS) | (1 << MAX_RT))) { // Завершена передача успехом, или нет,
    if (status & (1 << MAX_RT)) { // Если достигнуто максимальное число попыток
      radio_cmd(FLUSH_TX); // Удалим последний пакет из очереди
    }
    if (!(radio_readreg(FIFO_STATUS) & (1 << TX_EMPTY))) { // Если в очереди передатчика есть что передавать
      setCE(1); // Импульс на линии CE приведёт к началу передачи
      delay_us(15); // Нужно минимум 10мкс, возьмём с запасом
      setCE(0);
    } else {
      conf = radio_readreg(CONFIG);
      radio_writereg(CONFIG, conf | (1 << PRIM_RX)); // Устанавливаем бит PRIM_RX: приём
      setCE(1); // Высокий уровень на линии CE переводит радио-чип в режим приёма
    }
  }

  while (((status & (7 << RX_P_NO)) != (7 << RX_P_NO)) && protect--) { // Пока в очереди есть принятый пакет
    unsigned char l = radio_read_rx_payload_width(); // Узнаём длину пакета
    if (l > 32) { // Ошибка. Такой пакет нужно сбросить
      radio_cmd(FLUSH_RX);
    } else {
      unsigned char buf[32]; // буфер для принятого пакета
      radio_read_buf(R_RX_PAYLOAD, &buf[0], l); // начитывается пакет
      if ((status & (7 << RX_P_NO)) == (1 << RX_P_NO)) { // если datapipe 1
        on_packet(&buf[0], l); // вызываем обработчик полученного пакета
      }
    }
    status = radio_cmd(NOP);
  }
}



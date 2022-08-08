//Inclusoes.
#include "ch.h"
#include "hal.h"

//Definicoes.
#define PORTB IOPORT2         //PortB (Digitais, Utilizado).
#define PORTC IOPORT3         //PortC (Analógicos, Não Utilizado).
#define PORTD IOPORT4         //PortD (Digitais, Utilizado).

#define SENSOR_PE_D     2
#define SENSOR_VS_D     3

#define VP_VERMELHO_D   4
#define VP_AMARELO_D    5
#define VP_VERDE_D      6

#define VS_VERMELHO_D   7
#define VS_AMARELO_B    0
#define VS_VERDE_B      1

#define PE_VERMELHO_B   2
#define PE_VERDE_B      3

//Enumerar Estados.
typedef enum estado{
  Estado_0,
  Estado_1,
  Estado_2,
  Estado_3,
  Estado_4,
  Estado_5,
  Estado_6,
  Estado_7,
  Estado_8,
  Estado_9
}enum_estado;

//Global Sensores e Estado.
volatile uint8_t PE = 0;
volatile uint8_t VS = 0;
enum_estado Estado = Estado_0;

//Global Temporizadores Virtuais.
static virtual_timer_t vt_verde_vp;
static virtual_timer_t vt_amarelo_vp;
static virtual_timer_t vt_amarelo_vs;
static virtual_timer_t vt_amarelo_pe;

//Flags para os temporizadores não ficarem sendo chamados a todo momento e resetarem a contagem.
uint8_t Flag1 = 1;
uint8_t Flag2 = 1;
uint8_t Flag3 = 1;
uint8_t Flag4 = 1;

//Thread 1 - Verifica o Sensor do Pedestre
static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg){
  (void) arg;
  chRegSetThreadName("Sensor_PE");
  while(1){
    PE = !palReadPad(PORTD, SENSOR_PE_D);
    chThdSleepMilliseconds(10);
  }
}

//Thread 2 - Verifica o Sensor da Via Secundária
static THD_WORKING_AREA(waThread2, 32);
static THD_FUNCTION(Thread2, arg){
  (void) arg;
  chRegSetThreadName("Sensor_VS");
  while(1){
    VS = !palReadPad(PORTD, SENSOR_VS_D);
    chThdSleepMilliseconds(10);
  }
}

//Temporizador Virtual, Verde Via Principal
static void cb_verde_vp(void *arg){
  chSysLockFromISR();
  if(PE==1 || VS==1){
    Estado = Estado_2;
    Flag1 = 1;
  }else{
    chVTSetI(&vt_verde_vp, TIME_MS2I(1), cb_verde_vp, NULL);
  }
  chSysUnlockFromISR();
}

//Temporizador Virtual, Amarelo Via Principal
static void cb_amarelo_vp(void *arg){
  chSysLockFromISR();
  Estado = Estado_3;
  Flag2 = 1;
  chSysUnlockFromISR();
}

//Temporizador Virtual, Amarelo Via Secundaria
static void cb_amarelo_vs(void *arg){
  chSysLockFromISR();
  Estado = Estado_6;
  Flag3 = 1;
  chSysUnlockFromISR();
}

//Temporizador Virtual, Amarelo Via Pedestres
static void cb_amarelo_pe(void *arg){
  chSysLockFromISR();
  static uint8_t cont = 0;
  palTogglePad(PORTB, PE_VERMELHO_B);
  if(cont<3){
    chVTSetI(&vt_amarelo_pe, TIME_MS2I(500), cb_amarelo_pe, NULL);
    cont++;
  }else{
    cont = 0;
    Flag4 = 1;
    Estado = Estado_9;
  }
  chSysUnlockFromISR();
}

//Processamento dos Dados
processo(){
  switch (Estado){
    //Set inicial do semaforo.
    case Estado_0:
      palClearPad(PORTD, VP_VERMELHO_D);  // ○
      palClearPad(PORTD, VP_AMARELO_D);   // ○
      palSetPad(PORTD, VP_VERDE_D);       // ◉
      
      palSetPad(PORTD, VS_VERMELHO_D);    // ◉
      palClearPad(PORTB, VS_AMARELO_B);   // ○
      palClearPad(PORTB, VS_VERDE_B);     // ○

      palSetPad(PORTB, PE_VERMELHO_B);    // ◉
      palClearPad(PORTB, PE_VERDE_B);     // ○
    break;

    //Verde da via principal
    case Estado_1:
      palClearPad(PORTD, VP_VERMELHO_D);
      palSetPad(PORTD, VP_VERDE_D);
      if(Flag1){
        chVTSet(&vt_verde_vp, TIME_MS2I(4000), cb_verde_vp, NULL);
        Flag1 = 0;
      }
    break;

    //Amarelo da via principal.
    case Estado_2:
      palClearPad(PORTD, VP_VERDE_D);
      palSetPad(PORTD, VP_AMARELO_D);
      if(Flag2){
        chVTSet(&vt_amarelo_vp, TIME_MS2I(2000), cb_amarelo_vp, NULL);
        Flag2 = 0;
      }
    break;

    //Vermelho da via principal.
    case Estado_3:
      palClearPad(PORTD, VP_AMARELO_D);
      palSetPad(PORTD, VP_VERMELHO_D);
      if(PE){
        Estado = Estado_7;
      }else if(VS){
        Estado = Estado_4;
      }
    break;

    //Verde da via secundaria.
    case Estado_4:
      if(VS){
        palClearPad(PORTD, VS_VERMELHO_D);
        palSetPad(PORTB, VS_VERDE_B);
      }else{
        Estado = Estado_5;
      }
    break;

    //Amarelo da via secundaria.
    case Estado_5:
      palClearPad(PORTB, VS_VERDE_B);
      palSetPad(PORTB, VS_AMARELO_B);
      if(Flag3){
        chVTSet(&vt_amarelo_vs, TIME_MS2I(2000), cb_amarelo_vs, NULL);
        Flag3 = 0;
      }
    break;

    //Vermelho da via secundaria.
    case Estado_6:
      palClearPad(PORTB, VS_AMARELO_B);
      palSetPad(PORTD, VS_VERMELHO_D);
      Estado = Estado_1;
    break;

    //Verde dos pedesdres
    case Estado_7:
      if(PE){
        palClearPad(PORTB, PE_VERMELHO_B);
        palSetPad(PORTB, PE_VERDE_B);
      }else{
        Estado = Estado_8;
      }
    break;

    //Amarelo dos pedestres
    case Estado_8:
      palClearPad(PORTB, PE_VERDE_B);
      if(Flag4){
        palTogglePad(PORTB, PE_VERMELHO_B);
        chVTSet(&vt_amarelo_pe, TIME_MS2I(500), cb_amarelo_pe, NULL);
        Flag4 = 0;
      }
    break;

    //Vermelho dos pedestres
    case Estado_9:
      palSetPad(PORTB, PE_VERMELHO_B);
      if(VS){
        Estado = Estado_4;
      }else{
        Estado = Estado_1;
      }
    break;
  }
}

int main(void) {
  //Iniciar SO.
  halInit();
  chSysInit();

  //Setar modo dos pinos.
  palSetPadMode(PORTD, SENSOR_PE_D,    PAL_MODE_INPUT);
  palSetPadMode(PORTD, SENSOR_VS_D,    PAL_MODE_INPUT);
  palSetPadMode(PORTD, VP_VERMELHO_D,  PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(PORTD, VP_AMARELO_D,   PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(PORTD, VP_VERDE_D,     PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(PORTD, VS_VERMELHO_D,  PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(PORTB, VS_AMARELO_B,   PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(PORTB, VS_VERDE_B,     PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(PORTB, PE_VERMELHO_B,  PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(PORTB, PE_VERDE_B,     PAL_MODE_OUTPUT_PUSHPULL);

  //Criar Threads.
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  //Inicializar Objetos de Temporizadores Virtuais.
  chVTObjectInit(&vt_verde_vp);
  chVTObjectInit(&vt_amarelo_vp);
  chVTObjectInit(&vt_amarelo_vs);
  chVTObjectInit(&vt_amarelo_pe);

  //Estado inicial do semaforo em 0, depois atribuido 1.
  processo();
  Estado = Estado_1;

  //Laco infinito de processamento.
  while(true){
    processo();
    chThdSleepMilliseconds(10);
  }
}
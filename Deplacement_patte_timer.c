#include "LPC17xx.h"
#include "type.h"
#include "can.h"
#include "LED.h"
#include <math.h>
#include "UART.h"
#include <stdio.h>
#include "GPIO.h"

//adresse des cartes filles
#define UN 1
#define DEUX 7
#define TROIS 3
#define QUATRE 9
#define CINQ 5
#define SIX 6


#define fluidity 4 //2 fonctionne mais le temps de réponses de moteur étant aléatoire étant donné leur âge....

#define PWMPRESCALE (25-1) //25 PCLK cycles to increment TC by 1 i.e. 1 Micro-second

CAN_MSG MsgBuf_TX1, MsgBuf_TX2; /* TX and RX Buffers for CAN message */
CAN_MSG MsgBuf_RX1, MsgBuf_RX2; /* TX and RX Buffers for CAN message */
volatile uint32_t CAN1RxDone = FALSE, CAN2RxDone = FALSE;

void initPWM(void);
void CAN_Initialisation(void);
void Send_CAN(int ID, int DataA, int DataB);
void send_position(int ID, int positionA, int positionB, int positionC);

void test_servo(short patte);
void test_all_servo(void);

int tps_timer0=0,flag_timer0=0;
int tps_timer1=0,flag_timer1=0;
int k=0,patte=0,testeur=0;
// DataA : paquet le plus lourd. Initialisation du Baudarate à 125kBps
int servo[6][3];//npatte et n_servo

char etat_deplacement=0,etat=0;
char poubelle[2]={0};
short i=0,angle,distance,delay_rs;
char qualite, angle1,  angle2,  distance1 ,  distance2;
short diametre1 = 300, diametre2 = 600;
char tableau_reception_lidar[20]={0};

short test_bluetooth(void);
void send_BT(char * str,short nb_char);
void delay(int seconde);
void delay2(int seconde);
void rotation(int nombre_de_tour);
void synchro_patte_2_4_6(int pos_1,int pos_2,int pos_3);
void synchro_patte_1_3_5(int pos_1,int pos_2,int pos_3);
void marche(short direction);
void retract_stand(void);
void default_position(void);

void TIMER0_IRQHandler(void) {  
	//int i=0;
	LPC_TIM0->IR |= 1<<0;   	// baisse Flag associé à MR0
	tps_timer0+=1;
	if(tps_timer0==100){
		tps_timer0=0;
		flag_timer0=1;
	}
}

void TIMER1_IRQHandler(void) {  
	//int i=0;
	LPC_TIM1->IR |= 1<<0;   	// baisse Flag associé à MR0
	tps_timer1+=1;
	if(tps_timer1==10){
		tps_timer1=0;
		flag_timer1=1;
	}
}

void Timer012_Init(unsigned int prescaler, unsigned int valeur){
	LPC_SC->PCONP = LPC_SC->PCONP | 0x00C00006;   // enable Timer 0,1,2,3

	LPC_TIM0->PR =  prescaler;
	LPC_TIM0->MR0 = valeur; 

	LPC_TIM0->MCR = 3;		// reset counter si MR0=COUNTER + interrupt
	LPC_TIM0->TCR = 1; 		// démarre le comptage
	
	// parametrage timer 1
	
	LPC_TIM1->MCR  = 3;     /* Clear TC on MR0 match and Generate Interrupt*/
  LPC_TIM1->PR   =  prescaler;      /* Prescalar for 1us */
  LPC_TIM1->MR0  = 	valeur;                 /* Load timer value to generate 500ms delay*/
  LPC_TIM1->TCR  = 1;                    /* Start timer by setting the Counter Enable*/
	
	
}
// Programme principal
int main(void) {
	LPC_GPIO1->FIODIR0 |= 0x13;
	LPC_GPIO1->FIODIR1 |= 0xC7;	
	
	Init_UART();
	initPWM();	
	CAN_Initialisation();
	Timer012_Init(0, 32767);  	// prescaler = 0+1  diviseur -> 1s //32767 pour 1 ms
	NVIC_EnableIRQ(TIMER0_IRQn); 		// validation IT pour Timer 0 (bit 1)	
	NVIC_EnableIRQ(TIMER1_IRQn); 		// validation IT pour Timer 1 (bit 1)
	
	LPC_GPIO1->FIOPIN0 |= 0x13;
	LPC_GPIO1->FIOPIN1 |= 0xC7;
	delay(5);
	LPC_GPIO1->FIOPIN0 = 0x00;
	LPC_GPIO1->FIOPIN1 = 0x00;
	
	default_position();
	delay(20);
	
	UART_LIDAR_PutChar(0xA5);
	for(delay_rs=0;delay_rs<100;delay_rs++);
	UART_LIDAR_PutChar(0x20);
	///////test bluetooth
	UART_BT_PutChar('W');
	while(1){
		send_BT("AT+ROLE=<Param>",15);
		delay(1);
		send_BT("AT+ROLE?\r\n",9);
		delay(10);
		
		
	}/*{
		if(test_bluetooth()==1){
			LPC_GPIO1->FIODIR0 |= 0x13;
			LPC_GPIO1->FIODIR1 |= 0xC7;	
		}
	}*/
//while(1);
	///////fin test
	while(1)
	{
		switch(etat){
			case 0:	
					LPC_GPIO1->FIOPIN0 = 0x00;
					LPC_GPIO1->FIOPIN1 = 0x00;
					LPC_GPIO1->FIOPIN0 |= 0x01;
					tableau_reception_lidar[0] = UART_LIDAR_GetChar();
					tableau_reception_lidar[1] = UART_LIDAR_GetChar();
					tableau_reception_lidar[2] = UART_LIDAR_GetChar();
					tableau_reception_lidar[3] = UART_LIDAR_GetChar();
					tableau_reception_lidar[4] = UART_LIDAR_GetChar();
				
					//decodage de la trame pour obtenir les valeurs de qualité, distance, angle :
					//qualite = bit [7 à 2] de la trame 0
					qualite = tableau_reception_lidar[0];
					qualite = (qualite >>2);
					//angle = bit [14 à 7] de la trame 2 et bit [7 à 1] de la trame 1
					angle1= tableau_reception_lidar[1];
					angle2= tableau_reception_lidar[2];
					//valeur angle par concaténation [14:0]
					angle=((angle2<<7) | (angle1)>>1) / 64.00; // angle en degrés


					distance1 = tableau_reception_lidar[3];
					distance2 = tableau_reception_lidar[4];

					//distance = bit [15 à 8] de la trame 4 et bit [7 à 0] de la trame 3

					//valeur angle par concaténation [15:0]
					distance= ((distance2<<8) | distance1)/4.00;  //distance en mm
					//machine à état pour effectuer un déplacement en fonction des donnees lidar
					//do
					//Marche avant
					if ((distance <= diametre2) && (distance>100)){
						if ((angle <= 359) && (angle>=301))	//OBSTACLE EN AVANT GAUCHE
            {
							etat=1;
							LPC_GPIO1->FIOPIN0 |= 0x02;
            }
            else if  ((angle >=0) && (angle<=60 ))	//OBSTACLE EN AVANT DROITE
            {
              etat=2;
							LPC_GPIO1->FIOPIN0 |= 0x10;
            }
          }
					if ((distance <= diametre1) && (distance>100)){
						if ((angle <= 120) && (angle>=61)) // OBSTACLE à droite
            {
              etat=3;
							LPC_GPIO1->FIOPIN1 |= 0x01;
            }
            else if ((angle <= 180) && (angle>=121)) //OBSTACLE arrière droit
            {
              etat=4;
							LPC_GPIO1->FIOPIN1 |= 0x02;
            }
            else if ((angle <= 241) && (angle>=181)) //OBSTACLE arrière gauche
            {
              etat=5;
							LPC_GPIO1->FIOPIN1 |= 0x04;
            }
						else if((angle <= 300) && (angle >=242)) // OBSTACLE à gauche
						{
							etat=6;
							LPC_GPIO1->FIOPIN1 |= 0x40;
						}			
					}					
     break;
     case 1:
				LPC_GPIO1->FIOPIN0 |= 0x02;
				marche(2);//5
        etat = 0;
     break;
     case 2:
			  LPC_GPIO1->FIOPIN0 |= 0x10;
			  marche(3);//6
        etat = 0;
		 break;
     case 3:
				LPC_GPIO1->FIOPIN1 |= 0x01;
				marche(4);//1
        etat=0;
     break;
     case 4:
				LPC_GPIO1->FIOPIN1 |= 0x02;
				marche(5);//2
        etat=0;
     break;
     case 5:
				LPC_GPIO1->FIOPIN1 |= 0x04; 
				marche(6);//3
        etat=0;
     break;				
		 case 6:
				LPC_GPIO1->FIOPIN1 |= 0x40; 
				marche(1);//4
				etat = 0;
		 break;
     default:
				LPC_GPIO1->FIOPIN0 |= 0x13;
				LPC_GPIO1->FIOPIN1 |= 0xC7;
        etat =0;
     break;
   }
	}

return 0;	
}


void marche(short direction){
	short deux,quatre,six;
	short un,trois,cinq;
	
	if(direction==1){
		deux = UN;
		quatre = TROIS;
		six = CINQ;
		
		un = SIX;
		trois = DEUX;
		cinq = QUATRE;
	}
	else if (direction==2){
		deux = DEUX;//2
		quatre = QUATRE;//4
		six = SIX;//6
		
		un = UN;//1
		trois = TROIS;//3
		cinq = CINQ;//5
	}
	else if (direction==3){
		deux = TROIS;
		quatre = CINQ;
		six = UN;
		
		un = DEUX;
		trois = QUATRE;
		cinq = SIX;
	}
	else if (direction==4){
		deux = QUATRE;
		quatre = SIX;
		six = DEUX;
		
		un = TROIS;
		trois = CINQ;
		cinq = UN;
	}
	else if (direction==5){
		deux = CINQ;
		quatre = UN;
		six = TROIS;
		
		un = QUATRE;
		trois = SIX;
		cinq = DEUX;
	}
	else if (direction==6){
		deux = SIX;
		quatre = DEUX;
		six = QUATRE;
		
		un = CINQ;
		trois = UN;
		cinq = TROIS;
	}
	//direction vers l'avant = patte 2
	//mise en position 2 4 6 ################################################
	send_position(deux,150,70,210); 
	send_position(quatre,150,70,210); 
	send_position(six,150,70,210); 
	
	delay(fluidity);
	
	send_position(deux,150,70,140);
	send_position(quatre,190,70,150);
	send_position(six,110,70,150);
	
	delay(fluidity);
	
	send_position(deux,150,160,100); //direction <= direction%6+1 a faire avant
	send_position(quatre,190,150,150); //direction+2
	send_position(six,110,150,150); //direction+4
	
	//########################################################################
	//préparation pour mouvement 1 3 5#################################
	delay(fluidity);
	
	send_position(un,150,70,210); //direction-1
	send_position(trois,150,70,210); //direction+2-1
	send_position(cinq,150,70,210); //direction+4-1
	//#################################################
	
	
	
	//imprimer mouvement 2 4 6####################################################
	delay(fluidity);
	send_position(deux,150,150,150);
	send_position(quatre,150,150,150);
	send_position(six,150,150,150);
	//##########################################
	//fin préparation mouvement 1 3 5#####################################
	delay(fluidity);
	
	send_position(un,150,150,150);
	send_position(trois,150,150,150);
	send_position(cinq,150,150,150);
	
	delay(fluidity);
	//######################################################################
}
void rotation(int nombre_de_tour){
	int tour=0;
	for(tour=0;tour<nombre_de_tour;tour++){
		synchro_patte_1_3_5(150,70,190);
		delay(3);
		synchro_patte_2_4_6(150,150,150);
		
		delay(6);
		
		synchro_patte_1_3_5(190,150,150);
		delay(3);
		synchro_patte_2_4_6(150,70,190);
		
		delay(6);
		
		synchro_patte_1_3_5(150,150,150);
		delay(3);
		synchro_patte_2_4_6(190,150,150);
		
		delay(6);
		
		synchro_patte_1_3_5(150,70,150);
		delay(3);
		synchro_patte_2_4_6(150,150,150);
		
		delay(6);
		
		synchro_patte_1_3_5(150,150,150);
		
		delay(6);
	}
}

void synchro_patte_2_4_6(int pos_1,int pos_2,int pos_3){
	send_position(DEUX,pos_1,pos_2,pos_3);	
	send_position(QUATRE,pos_1,pos_2,pos_3);
	send_position(SIX,pos_1,pos_2,pos_3);
}
void synchro_patte_1_3_5(int pos_1,int pos_2,int pos_3){
	send_position(UN,pos_1,pos_2,pos_3);
	send_position(TROIS,pos_1,pos_2,pos_3);
	send_position(CINQ,pos_1,pos_2,pos_3);
}

void delay(int seconde){
	for(k=0;k<seconde;k++){
		while(flag_timer0==0);
		flag_timer0=0;
	}
}
void delay2(int seconde){
	for(k=0;k<seconde;k++){
		while(flag_timer1==0);
		flag_timer1=0;
	}
}
void CAN_Initialisation(void){
SystemClockUpdate();
CAN_Init( BITRATE125K18MHZ );
//CAN_SetACCF( ACCF_OFF );
}


void Send_CAN(int ID,int DataA, int DataB){
  /* Even though the filter RAM is set for all type of identifiers,
  the test module tests explicit standard identifier only */
  MsgBuf_TX1.Frame = 0x00080000; /* 11-bit, no RTR, DLC is 8 bytes */
  MsgBuf_TX1.MsgID = ID; /* Explicit Standard ID */
  MsgBuf_TX1.DataA = DataA;
  MsgBuf_TX1.DataB = DataB;
	
	
	//Transmit initial message on CAN 1 
	CAN1_SendMessage( &MsgBuf_TX1 );

}

void send_position(int ID, int positionA, int positionB, int positionC){
	int DataA_sp = 0x00, DataB_sp = 0x00;
	int ID_sp = ID+0x200;
	DataA_sp = DataA_sp | positionA;
	DataA_sp = DataA_sp<<4;

	DataB_sp = DataB_sp | positionB;
	DataB_sp = DataB_sp<<18;
	DataB_sp = DataB_sp | positionC;

	Send_CAN(ID_sp, DataA_sp, DataB_sp);

}


void retract_stand(void){
	send_position(UN,150,70,210);//1
	send_position(DEUX,150,70,210);//2
	send_position(TROIS,150,70,210);//3
	delay(1);
	send_position(QUATRE,150,70,210);//4
	send_position(CINQ,150,70,210);//5
	send_position(SIX,150,70,210);//6
}

void default_position(void){
	send_position(UN,150,150,150);
	send_position(DEUX,150,150,150);
	send_position(TROIS,150,150,150);
	delay(1);
	send_position(QUATRE,150,150,150);
	send_position(CINQ,150,150,150);
	send_position(SIX,150,150,150);
}
void test_servo(short patte){
	short i=0,j=0;
	short tab[3]={70};
	for(i=0;i<3;i++){
		for(j=70;j<211;j++){
			tab[i]=j;
			send_position(patte,tab[0],tab[1],tab[2]);
			delay2(1);
		}
		tab[i]=150;
	}
}

void test_all_servo(void){
		test_servo(UN);
		test_servo(DEUX);
		test_servo(TROIS);
		test_servo(QUATRE);
		test_servo(CINQ);
		test_servo(SIX);
	
	
}

void initPWM(void){	
	int rapport_cyclique = 60; // varie de 1 à 100
	// initialisation de timer 1
	LPC_SC->PCONP = LPC_SC->PCONP | 0x00000040;   // enable PWM1
	
	// prescaler+1 = 12 cela donne une horloge de base de période 480 ns
	// TC s'incrémente toutes les 480 ns
	LPC_PWM1->PR = 6;//11;  // prescaler 
	
	// valeur de MR0  + 1 = 100 cela fait 48 us, période de la PWM
  // valeur proche des 20 KHz de la gamelle !
  // ceci permet de régler facilement le rapport cyclique entre 1 et 100	

  LPC_PWM1->MR0 = 102;//99;    // Ceci ajuste la période de la PWM à 48 us 
	LPC_PWM1->MR3 = rapport_cyclique-1;    // ceci ajuste la duree de l'état haut
	
	LPC_PINCON->PINSEL7 = LPC_PINCON->PINSEL7 | 0x00300000; //  P3.26 est la sortie PWM Channel 3 de Timer 1
	
	LPC_PWM1->MCR = LPC_PWM1->MCR | 0x00000002; // Timer relancé quand MR0 repasse à 0
	LPC_PWM1->LER = LPC_PWM1->LER | 0x00000009;  // ceci donne le droit de modifier dynamiquement la valeur du rapport cyclique
	                                             // bit 0 = MR0    bit3 = MR3
	LPC_PWM1->PCR = LPC_PWM1->PCR | 0x00000e00;  // autorise la sortie PWM
	                                
  LPC_PWM1->TCR = 1;  /*validation de timer 1 et reset counter */
	
}

short test_bluetooth(void){
	short i,valid = 0;
	char rec_tab[3];
	char quit_tab[10]="---<enter>";
	send_BT("$$$",3);
	delay(11);
	for(i=0;i<3;i++)rec_tab[i]=UART_BT_GetChar();
	if(rec_tab[0]=='C' && rec_tab[0]=='M' && rec_tab[0]=='D'){
		send_BT("---<enter>",10);
		valid = 1;
	}
	
	return valid;
}
void send_BT(char * str,short nb_char){
	short i;
	for(i=0;i<nb_char;i++){
		UART_BT_PutChar(str[i]);
	}
}
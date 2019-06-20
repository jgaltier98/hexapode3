#include "LPC17xx.h"
#include "type.h"
#include "can.h"
#include "LED.h"
#include <math.h>
#include "UART.h"
#include <stdio.h>
#include "GPIO.h"

//adresse des cartes filles
#define ADDR_UN 1
#define ADDR_DEUX 7
#define ADDR_TROIS 3
#define ADDR_QUATRE 9
#define ADDR_CINQ 5
#define ADDR_SIX 6

//Fluidité des mouvements de marche
#define fluidity 4 //2 fonctionne mais le temps de réponses de moteur étant aléatoire étant donné leur âge....

CAN_MSG MsgBuf_TX1, MsgBuf_TX2; /* TX and RX Buffers for CAN message */
CAN_MSG MsgBuf_RX1, MsgBuf_RX2; /* TX and RX Buffers for CAN message */
volatile uint32_t CAN1RxDone = FALSE, CAN2RxDone = FALSE;
//Déclaration fonctions d'initialisation
void initPWM(void);
void CAN_Initialisation(void);
void Send_CAN(int ID, int DataA, int DataB);
void send_position(int ID, int positionA, int positionB, int positionC);
//Déclaration des variables et tableaux
short tab_addr_patte[6]={ADDR_UN,ADDR_DEUX,ADDR_TROIS,ADDR_QUATRE,ADDR_CINQ,ADDR_SIX};
short addr_index=0;
int tps_timer0=0,flag_timer0=0;
int tps_timer1=0,flag_timer1=0;
int k=0;
char get_octet_from_bt;
int servo[6][3];//nombre_de_patte et n_servo
char etat_deplacement=0,etat=0;
short i=0,angle,distance,delay_rs;
char qualite, angle1,  angle2,  distance1 ,  distance2;
short diametre1 = 300, diametre2 = 600;
char tableau_reception_lidar[20]={0};
//Déclaration des fonctions utilisées par l'hexapode
void test_servo(short patte);
void test_all_servo(void);
void send_BT(char * str,short nb_char);
void delay(int centieme_seconde);
void delay2(int dixieme_seconde);
void rotation(int nombre_de_tour);
void synchro_patte_2_4_6(int pos_1,int pos_2,int pos_3);
void synchro_patte_1_3_5(int pos_1,int pos_2,int pos_3);
void marche(short direction);
void retract_stand(void);
void default_position(void);

void TIMER0_IRQHandler(void) {  
	LPC_TIM0->IR |= 1<<0;   	// baisse Flag associé à MR0
	tps_timer0+=1;						//On compte les millisecondes
	if(tps_timer0==100){			//On attend 100 millisecondes
		tps_timer0=0;
		flag_timer0=1;
	}
}

void TIMER1_IRQHandler(void) {  
	LPC_TIM1->IR |= 1<<0;   	// baisse Flag associé à MR0
	tps_timer1+=1;
	if(tps_timer1==10){				//On attend 10 millisecondes
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
	//Configuration du registre des LEDs
	LPC_GPIO1->FIODIR0 |= 0x13;
	LPC_GPIO1->FIODIR1 |= 0xC7;	
	//Fin configuration du registre des leds
	//Initialisations des périphériques
	Init_UART();
	initPWM();	
	CAN_Initialisation();
	Timer012_Init(0, 32767);  	// prescaler = 0+1  diviseur -> 1s //32767 pour 1 ms
	NVIC_EnableIRQ(TIMER0_IRQn); 		// validation IT pour Timer 0 (bit 1)	
	NVIC_EnableIRQ(TIMER1_IRQn); 		// validation IT pour Timer 1 (bit 1)
	//Fin initialisation des périphériques
	//Test led
	LPC_GPIO1->FIOPIN0 |= 0x13;
	LPC_GPIO1->FIOPIN1 |= 0xC7;
	delay(5);
	LPC_GPIO1->FIOPIN0 = 0x00;
	LPC_GPIO1->FIOPIN1 = 0x00;
	//Fin test led
	//Position par défaut des pattes
	default_position();
	delay(20); //attentes 2 secondes
	//Trames d'activations du lidar
	UART_LIDAR_PutChar(0xA5);
	for(delay_rs=0;delay_rs<100;delay_rs++);
	UART_LIDAR_PutChar(0x20);
	//#####################################
	//#         Codes principales         #
	//#####################################
	while(1)
	{
		//Partie récupération des caractères du Bluetooth
		get_octet_from_bt = UART_BT_GetChar();
		if(get_octet_from_bt=='A'){	
			send_BT("test_servos",11);
			etat = 7;//testservo
		}
		else if(get_octet_from_bt=='B'){	
			send_BT("lidar_mode",10);
			etat = 0;//mode lidar
		}
		else if(get_octet_from_bt=='C'){	
			send_BT("stop",4);
			etat = 8;//arret
		}
		else if(get_octet_from_bt=='D'){	
			send_BT("rotation_mode",13);
			etat = 9;//mode rotation
		}
		//Fin partie récupération des caractères du Bluetooth
		//Début automate de contrôle de l'hexapode
		switch(etat){
			case 0:	
					//Extinction des LEDs
					LPC_GPIO1->FIOPIN0 = 0x00;
					LPC_GPIO1->FIOPIN1 = 0x00;
					//LEDs correspondantes  à l'état 0
					LPC_GPIO1->FIOPIN0 |= 0x01;
				  //Récupération de la trame envoyée par le lidar
					tableau_reception_lidar[0] = UART_LIDAR_GetChar();
					tableau_reception_lidar[1] = UART_LIDAR_GetChar();
					tableau_reception_lidar[2] = UART_LIDAR_GetChar();
					tableau_reception_lidar[3] = UART_LIDAR_GetChar();
					tableau_reception_lidar[4] = UART_LIDAR_GetChar();
				
					//Decodage de la trame pour obtenir les valeurs de qualité, distance, angle :
					//qualite = bit [7 à 2] de la trame 0
					qualite = tableau_reception_lidar[0];
					qualite = (qualite >>2);
					//Angle dans lequel est repérer l'obstacle
					//angle = bit [14 à 7] de la trame 2 et bit [7 à 1] de la trame 1
					angle1= tableau_reception_lidar[1];
					angle2= tableau_reception_lidar[2];
					//valeur angle par concaténation [14:0]
					angle=((angle2<<7) | (angle1)>>1) / 64.00; // angle en degrés
					//Récupération de la distance à laquelle est repérer l'obstacle
					//distance = bit [15 à 8] de la trame 4 et bit [7 à 0] de la trame 3
					distance1 = tableau_reception_lidar[3];
					distance2 = tableau_reception_lidar[4];
					//valeur angle par concaténation [15:0]
					distance= ((distance2<<8) | distance1)/4.00;  //distance en mm
					//Fin récupération de la trame envoyée par le lidar
					//##################################################
					//Conditions et instructions selon les informations récupérées
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
		 case 7:
			 test_servo(tab_addr_patte[addr_index]);
			 addr_index++;
			 if(addr_index>5) addr_index = 0;
		 break;
		 case 8:
		 break;
		 case 9:
			 rotation(1);
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
	//Fonction permettant à l'hexapode d'effectuer une fois l'action marche
	short deux,quatre,six;
	short un,trois,cinq;
	//Configuration des pattes suivants la direction choisi
	if(direction==1){
		deux = ADDR_UN;
		quatre = ADDR_TROIS;
		six = ADDR_CINQ;
		
		un = ADDR_SIX;
		trois = ADDR_DEUX;
		cinq = ADDR_QUATRE;
	}
	else if (direction==2){
		deux = ADDR_DEUX;
		quatre = ADDR_QUATRE;
		six = ADDR_SIX;
		
		un = ADDR_UN;
		trois = ADDR_TROIS;
		cinq = ADDR_CINQ;
	}
	else if (direction==3){
		deux = ADDR_TROIS;
		quatre = ADDR_CINQ;
		six = ADDR_UN;
		
		un = ADDR_DEUX;
		trois = ADDR_QUATRE;
		cinq = ADDR_SIX;
	}
	else if (direction==4){
		deux = ADDR_QUATRE;
		quatre = ADDR_SIX;
		six = ADDR_DEUX;
		
		un = ADDR_TROIS;
		trois = ADDR_CINQ;
		cinq = ADDR_UN;
	}
	else if (direction==5){
		deux = ADDR_CINQ;
		quatre = ADDR_UN;
		six = ADDR_TROIS;
		
		un = ADDR_QUATRE;
		trois = ADDR_SIX;
		cinq = ADDR_DEUX;
	}
	else if (direction==6){
		deux = ADDR_SIX;
		quatre = ADDR_DEUX;
		six = ADDR_QUATRE;
		
		un = ADDR_CINQ;
		trois = ADDR_UN;
		cinq = ADDR_TROIS;
	}
	//mise en position 2 4 6 ##########################################
	send_position(deux,150,70,210); 
	delay2(1);
	send_position(quatre,150,70,210); 
	delay2(1);
	send_position(six,150,70,210); 
	
	delay(fluidity);
	
	send_position(deux,150,70,140);
	delay2(1);
	send_position(quatre,190,70,150);
	delay2(1);
	send_position(six,110,70,150);
	
	delay(fluidity);
	
	send_position(deux,150,160,100); //direction <= direction%6+1 a faire avant
	delay2(1);
	send_position(quatre,190,150,150); //direction+2
	delay2(1);
	send_position(six,110,150,150); //direction+4
	
	//#################################################################
	//préparation pour mouvement 1 3 5#################################
	delay(fluidity);
	
	send_position(un,150,70,210); //direction-1
	delay2(1);
	send_position(trois,150,70,210); //direction+2-1
	delay2(1);
	send_position(cinq,150,70,210); //direction+4-1
	//#################################################################
	//imprimer mouvement 2 4 6#########################################
	delay(fluidity);
	send_position(deux,150,150,150);
	delay2(1);
	send_position(quatre,150,150,150);
	delay2(1);
	send_position(six,150,150,150);
	//#################################################################
	//fin préparation mouvement 1 3 5##################################
	delay(fluidity);
	
	send_position(un,150,150,150);
	delay2(1);
	send_position(trois,150,150,150);
	delay2(1); 
	send_position(cinq,150,150,150);
	
	delay(fluidity);
	//#################################################################
}
void rotation(int nombre_de_tour){
	//Fonction permettant à l'hexapode d'effectuer une fois l'action rotation
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
	//Fonction permettant de synchroniser les pattes 2,4 et 6
	send_position(ADDR_DEUX,pos_1,pos_2,pos_3);	
	delay(1);
	send_position(ADDR_QUATRE,pos_1,pos_2,pos_3);
	delay(1);
	send_position(ADDR_SIX,pos_1,pos_2,pos_3);
	delay(1);
}
void synchro_patte_1_3_5(int pos_1,int pos_2,int pos_3){
	//Fonction permettant de synchroniser les pattes 1,3 et 5
	send_position(ADDR_UN,pos_1,pos_2,pos_3);
	delay(1);
	send_position(ADDR_TROIS,pos_1,pos_2,pos_3);
	delay(1);
	send_position(ADDR_CINQ,pos_1,pos_2,pos_3);
	delay(1);
}

void delay(int centieme_seconde){
	//Attend le nombre de centième de secondes passé en paramètre
	for(k=0;k<centieme_seconde;k++){
		while(flag_timer0==0);
		flag_timer0=0;
	}
}
void delay2(int dixieme_seconde){
	//Attend le nombre de dixième de secondes passé en paramètre
	for(k=0;k<dixieme_seconde;k++){
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
	//L'hexapode rétracte toutes ses pattes
	send_position(ADDR_UN,150,70,210);//1
	send_position(ADDR_DEUX,150,70,210);//2
	send_position(ADDR_TROIS,150,70,210);//3
	delay(1);
	send_position(ADDR_QUATRE,150,70,210);//4
	send_position(ADDR_CINQ,150,70,210);//5
	send_position(ADDR_SIX,150,70,210);//6
}

void default_position(void){
	//Position par défaut de l'hexapode
	send_position(ADDR_UN,150,150,150);
	send_position(ADDR_DEUX,150,150,150);
	send_position(ADDR_TROIS,150,150,150);
	delay(1);
	send_position(ADDR_QUATRE,150,150,150);
	send_position(ADDR_CINQ,150,150,150);
	send_position(ADDR_SIX,150,150,150);
}
void test_servo(short patte){
	//Test la patte dont l'adresse est passée en paramètre
	short i=0,j=0;
	short tab[3]={70}; //mise en position des servos à leurs positions la plus faible
	for(i=0;i<3;i++){
		for(j=70;j<211;j++){
			tab[i]=j;
			send_position(patte,tab[0],tab[1],tab[2]);
			delay2(1);
		}
		tab[i]=150;
	}
	tab[0]=150;
	tab[1]=150;
	tab[2]=150;
	send_position(patte,tab[0],tab[1],tab[2]);
}

void test_all_servo(void){
	//Test toutes les pattes
		test_servo(ADDR_UN);
		test_servo(ADDR_DEUX);
		test_servo(ADDR_TROIS);
		test_servo(ADDR_QUATRE);
		test_servo(ADDR_CINQ);
		test_servo(ADDR_SIX);
	
	
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

void send_BT(char * str,short nb_char){
	//Envoie la série de caractère passé en paramètre
	//nb_char indique le nombre de caractère de la chaine
	short i;
	for(i=0;i<nb_char;i++){
		UART_BT_PutChar(str[i]);
	}
}
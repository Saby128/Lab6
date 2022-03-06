; Archivo:Postlab6.s
; Dispositivo: PIC16F887 
; Autor: Saby Andrade
; Copilador: pic-as (v2.30), MPLABX v5.40
;
; Programa: Timer 1
; Hardware: Incrementar una variable cada 1s y encendemos un led cada 500ms
;
; Creado: 27 de febrero , 2022
; Última modificación: 5 de marzo, 2022


PROCESSOR 16F887
    
; PIC16F887 Configuration Bit Settings

// config statements should precede project file includes.
#include <xc.inc>
    
; Assembly source line config statements

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

 ;--------------macros---------------------------------------------------
Timer0_Reset MACRO
    banksel TMR0
    movlw   250		    ; valor inicial/ delay suficiente se mueve al registro W
    movwf   TMR0	    ; Se configura que tenga 1.5ms de retardo
    bcf	    T0IF	    ;Limpia la bandera de interrupción
    endm
    
Timer1_Reset MACRO  TMR1_H, TMR1_L
    banksel TMR1H	    ;Se direcciona al banco
    movlw   TMR1_H	    ; valor inicial/ delay suficiente se mueve al registro W
    movwf   TMR1H	    ; Valor inicial se mueve al TMR1H
    movlw   TMR1_L	    ; Valor inciial se mueve al registro W
    movwf   TMR1L	    ; Valor inicial se mueve en TMR1L. Configurando que tenga 500ms de retardo
    bcf	    TMR1IF	    ; limpiar la bandera del timer para no sobre poner valores (interrupcion)   
    endm 
 
; ------- VARIABLES EN MEMORIA --------
PSECT udata_shr    ;comoon memory
    Temp_W:	    DS	1   ;1 byte
    Temp_Status:    DS	1   ;1 byte
   
PSECT udata_bank0
    Second_:	    DS	1	;1 byte
    Second_1:	    DS	1	;1byte
    Second_2:	    DS	1	;1 byte
    VAR:	    DS	1	;1byte
    Conta_dor:	    DS	1	;1byte
    Sis_Decen:	    DS	1	;1 byte
    Sis_Unida:	    DS	1	;1 byte
    Display_Bandera:DS  1	;1 byte
    Display_:	    DS  2	;2 byte


PSECT resVect, class=CODE, abs, delta=2
ORG 00h			    ; posición 0000h para el reset
;------------ VECTOR RESET --------------
resetVec:
    PAGESEL main	; Cambio de pagina
    GOTO    main
    
PSECT intVect, class=CODE, abs, delta=2
ORG 04h			    ; posición 0004h para interrupciones
;------- VECTOR INTERRUPCIONES ----------
PUSH:				; Se mueve la instruccion de la PC a pila
    MOVWF   Temp_W		; del registro W a la variable "Temp_W"	
    SWAPF   STATUS, W		; Swap de nibbles del STATUS y se almacena en el registro W
    MOVWF   Temp_Status    	; Rutina de interrupción
    
ISR:
    BTFSC   T0IF		;Cambio de bandera en el timer0 No=0 Si=1. Salta al estar apagada
    CALL    Timer0_Interrup	;Se llama la subrutina
    
    BTFSC   TMR1IF		;Se encuentra apagada la bandera de cambio en el Timer 1 se salta la línea, si está encendida lo analiza
    CALL    Timer1_Interrup	;Se llama la subrutina	
    
    BTFSC   TMR2IF		;Se encuentra apagada la bandera de cambio en el Timer 2 se salta la línea, si está encendida lo analiza
    CALL    Timer2_Interrup	;Se llama la subrutina	
	
    
POP:				; Se mueve la instrucción de la pila a la PC
    SWAPF   Temp_W, W		;Swap de nibbles de la variable "Temp_W" y se almacena en el registro W
    MOVWF   STATUS		;Se mueve el registro W a STATUS	
    SWAPF   Temp_W, F		;Swap de nibbles de la variable "Temp_W" y se almacena en la variable "Temp_W"
    SWAPF   Temp_W, W		;Swap de nibbles de la variable "Temp_W" y se almacena en el registro W
    RETFIE

; ------ SUBRUTINAS DE INTERRUPCIONES ------
Timer0_Interrup:
    Timer0_Reset		;Llama el macro que reinicia el Timer 0 en el tiempo de 50MS
    
    //Var_Mostrar es la subrutina que enseña el display
    CALL Var_Mostrar		; Se llama la subrutina
    RETURN			;Se regresa
    
Timer1_Interrup:
    //Esta macro reinicia el Timer 1 para 1s
    Timer1_Reset 0x0B, 0xDC	; Llamamos la macro
    INCF  Second_		;En la variable se incrementa en uno
    BTFSS Second_,1		;Se salta la línea si la variable es 1  en su segundo bit
    RETURN			;Se regresa
    
    //Esta variable se utiliza para la repetición de 2 cual es equivalente a 1s
    CLRF Second_		;Se limpia
    INCF Second_2		;En la variable se incrementa 1
    MOVF Second_2,W		; El valor de la variable se mueve al registro W
    MOVWF Conta_dor		; Del registro W se mueve a la variable Conta_dor
    MOVLW 100			;El valor 100 se mueve al registro W  
    
    SUBWF Conta_dor,F		;A la variable se le resta el valor de 100 y se almacena en Conta_dor
    BTFSC ZERO			;Se comprueba que se encuentre apagada la bandera de ZERO
    CLRF Second_2		;Se encuentra apagada no se resetea
    RETURN			;Se egresa
    
Timer2_Interrup:
    BCF	    TMR2IF		;La bandera de interrupcion se limpia
    INCF    VAR			;En la variable se incrementa en uno
    BTFSS   VAR, 1	        ;Se salta la línea si la variable es 1  en su segundo bit
    Return			;Se regresa
    
    BTFSS   VAR, 3		;Se salta la línea si la variable es 1  en su cuarto bit
    Return			;Se regresa
    
    CLRF    VAR			;Se limpia la variable
    INCF    PORTB		;En el puerto B se incrementa en 1
    Return			;Se regresa   
    
PSECT code, delta=2, abs
ORG 100h		    ; posición 100h para el codigo

;------------Tabla---------------------
ORG 200h
table:
    CLRF    PCLATH		; Limpiamos registro PCLATH
    BSF	    PCLATH, 1		; Posicionamos el PC en dirección 02xxh
    ANDLW   0x0F		; no saltar más del tamaño de la tabla
    ADDWF   PCL
    RETLW   00111111B	;0
    RETLW   00000110B	;1
    RETLW   01011011B	;2
    RETLW   01001111B	;3
    RETLW   01100110B	;4
    RETLW   01101101B	;5
    RETLW   01111101B	;6
    RETLW   00000111B	;7
    RETLW   01111111B	;8
    RETLW   01101111B	;9
    RETLW   01110111B	;A
    RETLW   01111100B	;b
    RETLW   00111001B	;C
    RETLW   01011110B	;d
    RETLW   01111001B	;E
    RETLW   01110001B	;F
    
 
 ;------------- CONFIGURACION ------------
main:
    CALL    IO_Config		;Se llama la subrutina de configuración de entradas /salidas	
    CALL    Reloj_Config	;Se llama la subrutina de configuración del reloj
    CALL    Timer0_Config	;Se llama la subrutina de configuración del TMR0
    CALL    Timer1_Config	;Se llama la subrutina de configuración del TMR1
    CALL    Timer2_Config	;Se llama la subrutina de configuración del TMR2
    CALL    Interrup_Config	;Se llama la subrutina de configuración de interrupciones
    BANKSEL PORTA	    ; Cambio a banco 00

;--------------Loop Principal-------------------
loop:
    ; Código que se va a estar ejecutando mientras no hayan interrupciones
    CALL    Display_Set	; Guardamos los valores a enviar en PORTC para mostrar valor en decimales
    CALL    Division_	; Obtenemos las centenas/decenas y unidades
    GOTO    loop	    
    
;------------- SUBRUTINAS ---------------
Reloj_Config:
    BANKSEL OSCCON		;Direcciona al banco 01	
    
    //S= 1, C=0
    BSF	    OSCCON, 0		;SCS en 1, se configura a reloj interno
    BSF	    OSCCON, 6		;Bit 6 en 1
    BSF	    OSCCON, 5		;Bit 5 en 1
    BCF	    OSCCON, 4		;Bit 4 en 0
    ; Con una frecuencia interna del oscilador  a 4MHZ (IRCF<2:0> -> 110 4MHz)
    RETURN			;Se regresa
    
    
Timer0_Config:		
    BANKSEL OPTION_REG		;Se direcciona al banco 01
    
  //S= 1, C=0
    BCF	    OPTION_REG, 5	;TMR0 como temporizador
    BCF	    OPTION_REG, 3	;Prescaler a TMR0
    BSF	    OPTION_REG, 2	;Bit 2 en 1
    BSF	    OPTION_REG, 1	;Bit 1 en 1
    BSF	    OPTION_REG, 0	;Bit 0 en 1
    ;Prescaler en 256
    Timer0_Reset		;Se llama la Macro
    RETURN			;Se regresa
    
; Configuramos el TMR0 para obtener un retardo de 50ms
  
    
Timer1_Config:
    banksel INTCON  
    bcf	    TMR1CS		;El reloj interno se habilita
    bcf	    T1OSCEN		;LP se apaga
    bsf	    T1CKPS1	
    bsf	    T1CKPS0
    
    ;Se encuentra en un prescaler de 1:8
    bcf	    TMR1GE		;Estará contando siempre en el timer1	
    bsf	    TMR1ON		;El timer1 se enciende
    Timer1_Reset 0x0B, 0xDC	;Se llama la macro
    //El tiempo de configuración es de 500 mS en el Timer1
    return
          
Timer2_Config:
    banksel PR2
    MOVLW   240			;La literal se mueve al registro W
    MOVWF   PR2			;En el Timer 2 la configuración del tiempo es de 50 milisegundos
	
    BANKSEL T2CON		;Direccionamiento
    bsf	    T2CKPS1		; Prescaler 1:0	
    bsf	    T2CKPS0
    bsf	    TOUTPS3		; colocaremos el postcaler de 16:0
    
    bsf	    TOUTPS2
    bsf	    TOUTPS1
    bsf	    TOUTPS0
    bsf	    TMR2ON		;El timer 2 se enciende
    
 
IO_Config:			
    BANKSEL ANSEL		;Direcciona al banco 11
    CLRF    ANSEL		;Entradas o salidas digitales
    CLRF    ANSELH		;Entradas o salidas digitales
    
    BANKSEL TRISA		;Se direcciona al banco 01
    BSF	    TRISB, 0		;RB0 Como entrada
    CLRF    TRISC		;El puerto C como salida
    CLRF    TRISD		;El puerto D como salida
    
    BANKSEL PORTB		;Se direcciona al banco 
    CLRF    PORTB		;Limpia el puerto A
    CLRF    PORTB		;Limpia el puerto B
    CLRF    PORTC		;Limpia el puerto C
    CLRF    PORTD		;Limpia el puerto D
    
    CLRF    Sis_Decen		;Limpia la variable "Sis_Decen"
    CLRF    Sis_Unida		;Limpia la variable "Sis_Unida"
    CLRF    Second_1		;Limpia la variable "Second_1"
    RETURN			;Se regresa  
    
    
Display_Set:			
    MOVF    Sis_Unida,	W	;De la variable "Sis_Unida" se mueve hacia el registro W
    CALL    table		;Se llama la tabla (para buscar el valor a cargar que se encuentra en el puerto C)
    MOVWF   Display_		;En una nueva variable llamada "Display_1" y se guarda
    
    MOVF    Sis_Decen, W	;De la variable "Sis_Decen" se mueve hacia el registro W
    CALL    table		 ;Se llama la tabla (para buscar el valor a cargar que se encuentra en el puerto C)
    MOVWF   Display_+1		 ;En una nueva variable llamada "Display_2" y se guarda
    
    RETURN			;Se regresa

Var_Mostrar:  //ENSEÑA		
    BCF	    PORTD,  0		;Display de niblle alto se limpia 
    BCF	    PORTD,  1		;Display de nibble bajo se limpia
    
    //La variable está en bit 0
    BTFSC   Display_Bandera,    0 ;Si está apagada se salta esta línea, encendida se verifica la línea 
    goto    Display_1		  ;Se mueve al display 
    
    
Display_0:
    MOVF    Display_,	W	;El valor se mueve de unidades hacia el registro W
    MOVWF   PORTC		;El valor de tabla se mueve al puerto C
    BSF	    PORTD,  1		;El display de nibble bajo se enciende
    BSF	    Display_Bandera,  0	;En la siguiente interrupcion se tiene un cambio de bandera al intercambiar con el segundo display
    RETURN			;Se regresa
       
Display_1:
    MOVF    Display_+1, W	;El valor se mueve de "Display+1" hacia el registro W
    MOVWF   PORTC		;El valor de tabla se mueve al puerto C 
    BSF	    PORTD,  0		;El display de nibble alto se enciende
    BCF	    Display_Bandera,  0 ;En la siguiente interrupcion se tiene un cambio de bandera al intercambiar con el segundo display
    RETURN			;Se regresa
    
Division_:
    BANKSEL PORTA		;Se direcciona al banco 00
    CLRF    Sis_Decen		;Limpia la variable "Sis_Decen"
    CLRF    Sis_Unida		;Limpia la variable "Sis_Unida"
    MOVF    Second_2, W		;El valor se mueve de la variable hacia el registro W
    MOVWF   Second_1
    
    MOVLW   10			;El valor de 10 se mueve hacia el registro W
    SUBWF   Second_1,  F	;Se guarda en la variable después de restar 10  a la variable
    INCF    Sis_Decen		;En la variable  se aumenta en 1
    
    //Apagado valor negativo, encendido valor positivo
    BTFSC   STATUS, 0		;Se comprueba que la bandera de BORRON este apagada 
    GOTO    $-4			;Se regresa 4 instrucciones previas al no estar apagada
    DECF    Sis_Decen		;Se decrece 1 a la variable "Sis_Decen" al estar apagada
    
    //Se vuelve a reevaluar el valor que tiene la variable después del incremento de valor
    MOVLW   10			;El valor de 10 se mueve hacia el registro W
    ADDWF   Second_1, F		;Para ser positivo, se añade los 10 al momento negativo que se encuentra la variable 
    CALL    Unidad_Obt		;Se llama la subrutina
    RETURN			;Se regresa
    
Unidad_Obt:	        ;    Ejemplo:
    ; Obtenemos UNIDADES
    MOVLW   1			;El valor de 1 se mueve hacia el registro de W
    SUBWF   Second_1,  F	;Se guarda en la variable  después de restar 1  a la variable 
    INCF    Sis_Unida		;En la variable "Sis_Unida" se aumenta en 1
   
  //Apagado valor negativo, encendido valor positivo
    BTFSC   STATUS, 0		;Se comprueba que la bandera de BORRON este apagada 
    GOTO    $-4			;Se regresa 4 instrucciones previas al no estar apagada
    DECF    Sis_Unida		;Se decrece 1 a la variable "Sis_Unida" al estar apagada
    
    //Se vuelve a reevaluar el valor que tiene la variable "Conta_dor" después del incremento de valor    
    MOVLW   1			;El valor de 1 se mueve hacia el registro W
    ADDWF   Second_1,  F	;Para ser positivo, se añade los 1 al momento negativo que se encuentra la variable 
    RETURN			;Se regresa
       
Interrup_Config:
    BANKSEL PIE1
    BSF	    TMR1IE		;En el timer 1 se habilitan las interrupciones
    BSF	    TMR2IE		;En el timer 2 se habilitan las interrupciones
    
    BANKSEL INTCON
  ; colocamos las banderas de habilitar interrupcion
    BSF	    GIE			;Las interrupciones globales se habilitan	
    bsf	    PEIE		;Las interrupciones periféricos se habilitan
    
    BSF	    T0IE		;La interrupción en el Timer0 se habilita
    BCF	    T0IF		;En el timer0 se limpia la bandera
    
    bsf	    TMR1IF		; En el timer 1 se limpia la bandera
    bsf	    TMR2IF		;En el timer 2 se limpia la bandera
    
    RETURN
       
    
END
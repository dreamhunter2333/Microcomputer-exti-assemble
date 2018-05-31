
BIT0   EQU 0X00000001
BIT2   EQU 0X00000004
BIT5   EQU 0X00000020 
BIT8   EQU 0X00000100
BIT9   EQU 0X00000200
BIT10  EQU 0X00000400
BIT11  EQU 0X00000800
LED0   EQU BIT8     ;LED3--PA.0
LED1   EQU BIT9     ;LED1--PA.1
LED2   EQU BIT10    ;LED2--PA.2
LED3   EQU BIT11
NVIC_ST_CTRL_R    EQU 0xE000E010
NVIC_ST_RELOAD_R  EQU 0xE000E014 
NVIC_ST_CURRENT_R EQU 0xE000E018
GPIOD      EQU 0X40011400  ;GPIOA 地址
GPIOD_CRL  EQU 0X40011400  ;低配置寄存器
GPIOD_CRH  EQU 0X40011404  ;高配置寄存器
GPIOD_ODR  EQU 0X4001140C  ;输出，偏移地址0Ch
GPIOD_BSRR EQU 0X40011410  ;低置位，高清除偏移地址10h
GPIOD_BRR  EQU 0X40011414  ;清除，偏移地址14h
IOPDEN     EQU BIT5     ;GPIOA使能位
IOPAEN EQU BIT2 ;GPIOA使能位
KEY EQU BIT0 ;按键在PA.0
GPIOA EQU 0X40010800
GPIOA_CRL  EQU 0X40010800  ;低配置寄存器
GPIOA_IDR  EQU 0X40010808
RCC_APB2ENR EQU 0X40021018

STACK_TOP EQU 0X20002000
 AREA RESET,CODE,READONLY   ;AREA不能顶格写
 DCD STACK_TOP ;MSP主堆栈指针
 DCD START   ;复位，PC初始值 
 	 
 ENTRY         ;指示开始执行,ENTRY必须要顶格写
START                      ;所有的标号必须顶格写，且无冒号
 BL.W   RCC_CONFIG_72MHZ ;指令不能顶格写
 BL.W   SysTick_Init
 LDR    R1,=RCC_APB2ENR
 LDR    R0,[R1]     ;读
 LDR    R2,=IOPDEN
 ORR    R0,R2  ;改
 LDR    R2,=IOPAEN
 ORR    R0,R2		;改
 STR    R0,[R1]  ;写，使能GPIOA时钟
;LED0--PA.0 推挽输出，50MHz
;LED1--PA.1  推挽输出，50MHz
;LED2--PA.2  推挽输出，50MHz
 LDR    R0,=0x3333
 LDR    R1,=GPIOD_CRH  ;PA.1\2\3均在低寄存器
 STR    R0,[R1]
 NOP
 MOV R0,#0X04
 LDR R1,=GPIOA_CRL
 STR    R0,[R1]

 LDR    R1,=GPIOD_ODR
 LDR    R2,=0x00000F00 ;将PA.1\2\3输出高电平
 MOV    R3,#1
 LDR    R4,=GPIOA_IDR ;R4用于扫描按键
LOOP
 LDR    R5,[R4]
 AND.W  R5,#KEY
 CMP    R5,#0
 BNE	   LOOP
 PUSH   {R0}
 LDR    R0,=720000
 BL.W   SysTick_Wait ;延时10ms用于消除抖动
 POP    {R0}
 LDR    R5,[R4]
 AND.W  R5,#KEY
 CMP    R5,#0
 BNE    LOOP
 BL.W   LEDLED
 STR    R0,[R1]
WAIT_TO_UP		   ;等待按键弹起
 LDR    R5,[R4]
 AND.W  R5,#KEY
 CMP    R5,#0
 BEQ	   WAIT_TO_UP
 B      LOOP	
LEDLED
 STR    R2,[R1]
 LDR    R0,=72000000
 BL.W   SysTick_Wait
 EOR    R2,#LED0  ;翻转LED3
 ADD    R3,#1  ;计数加1

 CMP    R3,#1
 BEQ    STATE1
 CMP    R3,#3
 BEQ    STATE2
 CMP    R3,#5
 BEQ    STATE3
 CMP    R3,#7
 BEQ    STATE4
 CMP    R3,#8
 BEQ    STATE6
 CMP    R3,#10
 BEQ    STATE5
 B      GOON    

STATE1    ;状态1
 EOR    R2,#LED1
 EOR    R2,#LED2
 B      GOON
STATE2    ;状态2
 EOR    R2,#LED2
 B      GOON
STATE3    ;状态3
 EOR    R2,#LED1
 EOR    R2,#LED2
 B      GOON
STATE4    ;状态4
 EOR    R2,#LED2
 B      GOON
STATE5    ;状态5
 MOV    R3,#0
STATE6    ;状态4
 EOR    R2,#LED3
 B      GOON
GOON  
 B      LOOP    ;继续循环

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;RCC  时钟配置 HCLK=72MHz=HSE*9
;;;PCLK2=HCLK  PCLK1=HCLK/2
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
RCC_CONFIG_72MHZ
 LDR    R1,=0X40021000 ;RCC_CR
 LDR    R0,[R1]
 LDR    R2,=0X00010000 ;HSEON
 ORR    R0,R2
 STR    R0,[R1]
WAIT_HSE_RDY
 LDR    R2,=0X00020000 ;HSERDY
 LDR    R0,[R1]
 ANDS   R0,R2
 CMP    R0,#0
 BEQ    WAIT_HSE_RDY
 LDR    R1,=0X40022000 ;FLASH_ACR
 MOV    R0,#0X12
 STR    R0,[R1]
 LDR    R1,=0X40021004 ;RCC_CFGR时钟配置寄存器
 LDR    R0,[R1]
;PLL倍频系数,PCLK2,PCLK1分频设置
;HSE 9倍频PCLK2=HCLK,PCLK1=HCLK/2
;HCLK=72MHz 0x001D0400
;HCLK=64MHz 0x00190400
;HCLK=48MHz 0x00110400
;HCLK=32MHz 0x00090400
;HCLK=24MHz 0x00050400
;HCLK=16MHz 0x00010400
 LDR    R2,=0x001D0400 
 ORR    R0,R2
 STR    R0,[R1]
 LDR    R1,=0X40021000 ;RCC_CR  
 LDR    R0,[R1]
 LDR    R2,=0X01000000 ;PLLON
 ORR    R0,R2
 STR    R0,[R1]
WAIT_PLL_RDY
 LDR    R2,=0X02000000 ;PLLRDY
 LDR    R0,[R1]
 ANDS   R0,R2
 CMP    R0,#0
 BEQ    WAIT_PLL_RDY
 LDR    R1,=0X40021004 ;RCC_CFGR
 LDR    R0,[R1]
 MOV    R2,#0X02
 ORR    R0,R2
 STR    R0,[R1]
WAIT_HCLK_USEPLL
 LDR    R0,[R1]
 ANDS   R0,#0X08
 CMP    R0,#0X08
 BNE    WAIT_HCLK_USEPLL
 BX LR  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;延时R0（ms），误差((R0-1)*4+12)/8 us
;延时较长时，误差小于0.1%
DELAY_NMS
 PUSH   {R1}   ;2个周期
DELAY_NMSLOOP
 SUB    R0,#1
 MOV    R1,#1000
DELAY_ONEUS
    SUB    R1,#1
 NOP
 NOP
 NOP
 CMP    R1,#0
 BNE    DELAY_ONEUS
 CMP    R0,#0
 BNE    DELAY_NMSLOOP
 POP    {R1}
 BX     LR

SysTick_Init
; disable SysTick during setup
    LDR R1, =NVIC_ST_CTRL_R
    MOV R0, #0            ; Clear Enable         
    STR R0, [R1] 
; set reload to maximum reload value
    LDR R1, =NVIC_ST_RELOAD_R 
    LDR R0, =0x00FFFFFF;    ; Specify RELOAD value
    STR R0, [R1]            ; reload at maximum       
; writing any value to CURRENT clears it
    LDR R1, =NVIC_ST_CURRENT_R 
    MOV R0, #0              
    STR R0, [R1]            ; clear counter
; enable SysTick with core clock
    LDR R1, =NVIC_ST_CTRL_R    
    MOV R0, #0x0005    ; Enable but no interrupts (later)
    STR R0, [R1]       ; ENABLE and CLK_SRC bits set
    BX  LR 

SysTick_Wait
    SUB  R0, R0, #1   ; delay-1
    LDR  R6, =NVIC_ST_RELOAD_R  
    STR  R0, [R6]     ; time to wait
    LDR  R6, =NVIC_ST_CURRENT_R  
    STR  R0, [R6]     ; any value written to CURRENT clears
    LDR  R6, =NVIC_ST_CTRL_R  
SysTick_Wait_loop
    LDR  R0, [R6]     ; read status
    ANDS R0, R0, #0x00010000 ; bit 16 is COUNT flag
    BEQ  SysTick_Wait_loop   ; repeat until flag set
    BX   LR 

    NOP
    END
 
 
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
NVIC_ISER0 EQU 0xE000E100 ;bit 6 enables EXTI0
NVIC_IPR1 EQU 0xE000E404 ;3rd byte = EXTI0 priority
GPIOD      EQU 0X40011400  ;GPIOA 地址
GPIOD_CRL  EQU 0X40011400  ;低配置寄存器
GPIOD_CRH  EQU 0X40011404  ;高配置寄存器
GPIOD_ODR  EQU 0X4001140C  ;输出，偏移地址0Ch
GPIOD_BSRR EQU 0X40011410  ;低置位，高清除偏移地址10h
GPIOD_BRR  EQU 0X40011414  ;清除，偏移地址14h
IOPDEN     EQU BIT5     ;GPIOA使能位
IOPAEN EQU BIT2 ;GPIOA使能位
GPIOA EQU 0X40010800
GPIOA_CRL  EQU 0X40010800  ;低配置寄存器
GPIOA_IDR  EQU 0X40010808
RCC_APB2ENR EQU 0X40021018

	
NVIC_BASE               EQU             0xE000E000
NVIC_SETEN              EQU             (NVIC_BASE + 0x0010)    ;SETENA寄存器阵列的起始地址
NVIC_IRQPRI             EQU             (NVIC_BASE + 0x0400)    ;中断优先级寄存器阵列的起始地址
NVIC_VECTTBL            EQU             (NVIC_BASE + 0x0D08)    ;向量表偏移寄存器的地址
NVIC_AIRCR              EQU             (NVIC_BASE + 0x0D0C)    ;应用程序中断及复位控制寄存器的地址

NVIC_ST_CTRL_R    EQU 0xE000E010
NVIC_ST_RELOAD_R  EQU 0xE000E014 
NVIC_ST_CURRENT_R EQU 0xE000E018

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp               ; Top of Stack
                DCD     Reset_Handler              ; Reset Handler
                DCD     NMI_Handler                ; NMI Handler
                DCD     HardFault_Handler          ; Hard Fault Handler
                DCD     MemManage_Handler          ; MPU Fault Handler
                DCD     BusFault_Handler           ; Bus Fault Handler
                DCD     UsageFault_Handler         ; Usage Fault Handler
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     SVC_Handler                ; SVCall Handler
                DCD     DebugMon_Handler           ; Debug Monitor Handler
                DCD     0                          ; Reserved
                DCD     PendSV_Handler             ; PendSV Handler
                DCD     SysTick_Handler            ; SysTick Handler

                ; External Interrupts
                DCD     WWDG_IRQHandler            ; Window Watchdog
                DCD     PVD_IRQHandler             ; PVD through EXTI Line detect
                DCD     TAMPER_IRQHandler          ; Tamper
                DCD     RTC_IRQHandler             ; RTC
                DCD     FLASH_IRQHandler           ; Flash
                DCD     RCC_IRQHandler             ; RCC
                DCD     EXTI0_IRQHandler           ; EXTI Line 0
                DCD     EXTI1_IRQHandler           ; EXTI Line 1
                DCD     EXTI2_IRQHandler           ; EXTI Line 2
                DCD     EXTI3_IRQHandler           ; EXTI Line 3
                DCD     EXTI4_IRQHandler           ; EXTI Line 4
                DCD     DMA1_Channel1_IRQHandler   ; DMA1 Channel 1
                DCD     DMA1_Channel2_IRQHandler   ; DMA1 Channel 2
                DCD     DMA1_Channel3_IRQHandler   ; DMA1 Channel 3
                DCD     DMA1_Channel4_IRQHandler   ; DMA1 Channel 4
                DCD     DMA1_Channel5_IRQHandler   ; DMA1 Channel 5
                DCD     DMA1_Channel6_IRQHandler   ; DMA1 Channel 6
                DCD     DMA1_Channel7_IRQHandler   ; DMA1 Channel 7
                DCD     ADC1_2_IRQHandler          ; ADC1_2
                DCD     USB_HP_CAN1_TX_IRQHandler  ; USB High Priority or CAN1 TX
                DCD     USB_LP_CAN1_RX0_IRQHandler ; USB Low  Priority or CAN1 RX0
                DCD     CAN1_RX1_IRQHandler        ; CAN1 RX1
                DCD     CAN1_SCE_IRQHandler        ; CAN1 SCE
                DCD     EXTI9_5_IRQHandler         ; EXTI Line 9..5
                DCD     TIM1_BRK_IRQHandler        ; TIM1 Break
                DCD     TIM1_UP_IRQHandler         ; TIM1 Update
                DCD     TIM1_TRG_COM_IRQHandler    ; TIM1 Trigger and Commutation
                DCD     TIM1_CC_IRQHandler         ; TIM1 Capture Compare
                DCD     TIM2_IRQHandler            ; TIM2
                DCD     TIM3_IRQHandler            ; TIM3
                DCD     TIM4_IRQHandler            ; TIM4
                DCD     I2C1_EV_IRQHandler         ; I2C1 Event
                DCD     I2C1_ER_IRQHandler         ; I2C1 Error
                DCD     I2C2_EV_IRQHandler         ; I2C2 Event
                DCD     I2C2_ER_IRQHandler         ; I2C2 Error
                DCD     SPI1_IRQHandler            ; SPI1
                DCD     SPI2_IRQHandler            ; SPI2
                DCD     USART1_IRQHandler          ; USART1
                DCD     USART2_IRQHandler          ; USART2
                DCD     USART3_IRQHandler          ; USART3
                DCD     EXTI15_10_IRQHandler       ; EXTI Line 15..10
                DCD     RTCAlarm_IRQHandler        ; RTC Alarm through EXTI Line
                DCD     USBWakeUp_IRQHandler       ; USB Wakeup from suspend
__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

				ENTRY
; Reset handler
Reset_Handler    PROC
                 EXPORT  Reset_Handler             [WEAK]
START					 
				 BL.W   RCC_CONFIG_72MHZ ;指令不能顶格写
				 BL.W   SysTick_Init
				 LDR  R0,  =0xE000ED0C  ; 应用程序中断及复位控制寄存器  
				 LDR  R1,   =0x05FA0500  ; 使用优先级组5 (2/6)  
				 STR  R1,  [R0]    ; 设置优先级组  ...  
				 MOV  R4,  #8     ; ROM中的向量表  
				 LDR  R5,  =(__Vectors+8)  
				 LDMIA  R4!, {R0-R1}   ; 读取NMI和硬fault的向量    
				 STMIA  R5!, {R0-R1}   ; 拷贝它们的向量到新表中  ...  
				 LDR  R0,  =0xE000ED08  ; 向量表偏移量寄存器的地址  
				 LDR  R1,  =__Vectors 
				 STR  R1,  [R0]    ; 把向量表重定位  ...  
				 LDR  R0,  =EXTI0_IRQHandler ; 取得IRQ #7服务例程的入口地址  
				 LDR  R1,  =0xE000ED08  ; 向量表偏移量寄存器的地址  
				 LDR  R1,  [R1]  
				 ADD  R1,  R1,#(4*(7+16)); 计算IRQ #7服务例程的入口地址    
				 STR  R0,  [R1]    ; 在向量表中写入IRQ #7服务例程的入口地址  ...  
				 LDR  R0,  =0xE000E400  ; 外部中断优先级寄存器阵列的基地址  
				 MOV  R1,  #0xC0  
				 STRB  R1, [R0,#7]   ; 把IRQ #7的优先级设置为0xC0  ...  
				 LDR  R0,  =0xE000E100  ; SETEN寄存器的地址  
				 MOV  R1,  #(1<<7)   ; 置位IRQ #7的使能位  
				 STR  R1,  [R0]    ; 使能IRQ #7  
 LDR    R1,=RCC_APB2ENR
 LDR    R0,[R1]     ;读
 LDR    R2,=IOPDEN
 ORR    R0,R2  ;改
 STR    R0,[R1]  ;写，使能GPIOA时钟
;LED0--PA.0 推挽输出，50MHz
;LED1--PA.1  推挽输出，50MHz
;LED2--PA.2  推挽输出，50MHz
 LDR    R0,=0x3333
 LDR    R1,=GPIOD_CRH  ;PA.1\2\3均在低寄存器
 STR    R0,[R1]
 NOP
 NOP
 LDR    R1,=GPIOD_ODR
 LDR    R2,=0x00000F00 ;将PA.1\2\3输出高电平
 MOV    R3,#1
LOOP
 STR    R2,[R1]
 LDR    R0,=7200000  ;1125/9=125ms
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
                 ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler          [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler          [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler           [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler           [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler             [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler            [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  WWDG_IRQHandler            [WEAK]
                EXPORT  PVD_IRQHandler             [WEAK]
                EXPORT  TAMPER_IRQHandler          [WEAK]
                EXPORT  RTC_IRQHandler             [WEAK]
                EXPORT  FLASH_IRQHandler           [WEAK]
                EXPORT  RCC_IRQHandler             [WEAK]
                EXPORT  EXTI0_IRQHandler           [WEAK]
                EXPORT  EXTI1_IRQHandler           [WEAK]
                EXPORT  EXTI2_IRQHandler           [WEAK]
                EXPORT  EXTI3_IRQHandler           [WEAK]
                EXPORT  EXTI4_IRQHandler           [WEAK]
                EXPORT  DMA1_Channel1_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel2_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel3_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel4_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel5_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel6_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel7_IRQHandler   [WEAK]
                EXPORT  ADC1_2_IRQHandler          [WEAK]
                EXPORT  USB_HP_CAN1_TX_IRQHandler  [WEAK]
                EXPORT  USB_LP_CAN1_RX0_IRQHandler [WEAK]
                EXPORT  CAN1_RX1_IRQHandler        [WEAK]
                EXPORT  CAN1_SCE_IRQHandler        [WEAK]
                EXPORT  EXTI9_5_IRQHandler         [WEAK]
                EXPORT  TIM1_BRK_IRQHandler        [WEAK]
                EXPORT  TIM1_UP_IRQHandler         [WEAK]
                EXPORT  TIM1_TRG_COM_IRQHandler    [WEAK]
                EXPORT  TIM1_CC_IRQHandler         [WEAK]
                EXPORT  TIM2_IRQHandler            [WEAK]
                EXPORT  TIM3_IRQHandler            [WEAK]
                EXPORT  TIM4_IRQHandler            [WEAK]
                EXPORT  I2C1_EV_IRQHandler         [WEAK]
                EXPORT  I2C1_ER_IRQHandler         [WEAK]
                EXPORT  I2C2_EV_IRQHandler         [WEAK]
                EXPORT  I2C2_ER_IRQHandler         [WEAK]
                EXPORT  SPI1_IRQHandler            [WEAK]
                EXPORT  SPI2_IRQHandler            [WEAK]
                EXPORT  USART1_IRQHandler          [WEAK]
                EXPORT  USART2_IRQHandler          [WEAK]
                EXPORT  USART3_IRQHandler          [WEAK]
                EXPORT  EXTI15_10_IRQHandler       [WEAK]
                EXPORT  RTCAlarm_IRQHandler        [WEAK]
                EXPORT  USBWakeUp_IRQHandler       [WEAK]

WWDG_IRQHandler
PVD_IRQHandler
TAMPER_IRQHandler
RTC_IRQHandler
FLASH_IRQHandler
RCC_IRQHandler
EXTI0_IRQHandler
EXTI1_IRQHandler
EXTI2_IRQHandler
EXTI3_IRQHandler
EXTI4_IRQHandler
DMA1_Channel1_IRQHandler
DMA1_Channel2_IRQHandler
DMA1_Channel3_IRQHandler
DMA1_Channel4_IRQHandler
DMA1_Channel5_IRQHandler
DMA1_Channel6_IRQHandler
DMA1_Channel7_IRQHandler
ADC1_2_IRQHandler
USB_HP_CAN1_TX_IRQHandler
USB_LP_CAN1_RX0_IRQHandler
CAN1_RX1_IRQHandler
CAN1_SCE_IRQHandler
EXTI9_5_IRQHandler
TIM1_BRK_IRQHandler
TIM1_UP_IRQHandler
TIM1_TRG_COM_IRQHandler
TIM1_CC_IRQHandler
TIM2_IRQHandler
TIM3_IRQHandler
TIM4_IRQHandler
I2C1_EV_IRQHandler
I2C1_ER_IRQHandler
I2C2_EV_IRQHandler
I2C2_ER_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
USART1_IRQHandler
USART2_IRQHandler
USART3_IRQHandler
EXTI15_10_IRQHandler
RTCAlarm_IRQHandler
USBWakeUp_IRQHandler

                B       .

                ENDP

                ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                 IF      :DEF:__MICROLIB           
                
                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit
                
                 ELSE
                
                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap

                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR

                 ALIGN
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


                 ENDIF

                 END

;******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE*****

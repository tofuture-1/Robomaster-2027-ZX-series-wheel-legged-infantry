
;    PRESERVE8
;    THUMB

;    AREA    |.text|, CODE, READONLY
;    EXPORT  HardFault_Handler
;    IMPORT  HardFault_Handler_C

;HardFault_Handler PROC
;    ; ---------------------------------------------------------
;    ; 关键逻辑：查看 LR (EXC_RETURN) 确定死机时用的哪个栈
;    ; ---------------------------------------------------------
;    TST     LR, #4          ; 测试 LR 的 Bit 2
;    ITE     EQ              ; 判断
;    MRSEQ   R0, MSP         ; 如果 Bit 2 == 0，说明死机时在用 MSP
;    MRSNE   R0, PSP         ; 如果 Bit 2 == 1，说明死机时在用 PSP

;    ; ---------------------------------------------------------
;    ; 此时 R0 保存了正确的栈顶指针
;    ; 将 R0 作为参数传递给 C 函数 HardFault_Handler_C(uint32_t *stack)
;    ; ---------------------------------------------------------
;    B       HardFault_Handler_C
;    
;    ALIGN
;    ENDP

;    END

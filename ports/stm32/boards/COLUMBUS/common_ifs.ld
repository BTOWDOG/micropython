/* Memory layout for internal flash storage configuration:

    FLASH_ISR   .isr_vector

    FLASH_TEXT  .text
    FLASH_TEXT  .data

    RAM         .data
    RAM         .bss
    RAM         .heap
    RAM         .stack
*/

ENTRY(Reset_Handler)

REGION_ALIAS("FLASH_ISR", FLASH_START);
REGION_ALIAS("FLASH_COMMON", FLASH_TEXT);

/* define output sections */
SECTIONS
{
    INCLUDE common_isr.ld
    INCLUDE common_isr_extratext.ld
    INCLUDE common_text.ld
    INCLUDE common_extratext_data_in_flash.ld


    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >RAM

    /* This is to define the start of the heap, and make sure there is a minimum size */
    .heap :
    {
        . = ALIGN(4);
        . = . + _minimum_heap_size;
        . = ALIGN(4);
    } >RAM

    /* This checks there is enough RAM for the stack */
    .stack :
    {
        . = ALIGN(4);
        . = . + _minimum_stack_size;
        . = ALIGN(4);
    } >RAM

/* _bl_state symbol is used by MICROPY_HW_ENTER_BOOTLOADER_VIA_RESET, this is
   the end of stack address but is accessed as a different type. */
    _bl_state = _estack;
}


/* Platform selection is explicit. The host backend is never selected implicitly. */
#ifndef MB_HOST_BUILD
#error "No target HAL selected. Implement platform_microwatt.c against the finalized VHDL register map."
#endif
typedef int mb_platform_selection_translation_unit_t;

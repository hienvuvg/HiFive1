#include "mywdog.h"

// Input: watchdog counter duration in millisecond
void WDOG_Config(int ms){
  AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; AON_REG(AON_WDOGCFG) &= ~(1 << 0); 
  AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; AON_REG(AON_WDOGCFG) &= ~(1 << 1); 
  AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; AON_REG(AON_WDOGCFG) &= ~(1 << 2); 
  AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; AON_REG(AON_WDOGCFG) &= ~(1 << 3); 
  //AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; AON_REG(AON_WDOGCFG) |= AON_WDOGCFG_SCALE; // Counter scale value from wdogcount to wdogs; wgods increases every second
  AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; AON_REG(AON_WDOGCFG) |= 0x05; // Counter scale value from wdogcount to wdogs; wgods increases every millisecond

  AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; AON_REG(AON_WDOGFEED) = AON_WDOGFEED_VALUE; // Reset wdogcount to zero, worked

  AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; AON_REG(AON_WDOGCMP) = ms;// 16 bits, worked

  AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; AON_REG(AON_WDOGCFG) |= AON_WDOGCFG_ENALWAYS; // Enable watchdog, worked

  AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; AON_REG(AON_WDOGCFG) |= AON_WDOGCFG_RSTEN; // Controls whether the comparator output can set the wdogrst bit and hence cause a full reset.
}

void WDOG_Reset(void){
	AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; AON_REG(AON_WDOGFEED) = AON_WDOGFEED_VALUE; // Reset wdogcount to zero, worked
}
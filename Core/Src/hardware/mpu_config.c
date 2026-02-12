#include "mpu_config.h"

#if defined(__ICCARM__)
#pragma section="HEAP"
#define HEAP_START   ((uint32_t) __sfb("HEAP"))
#define HEAP_END      ((uint32_t) __sfe("HEAP"))
#elif defined(__ARMCC_VERSION)
extern uint32_t Image$$ARM_LIB_HEAP$$ZI$$Base;
extern uint32_t Image$$ARM_LIB_HEAP$$ZI$$Limit;
#define HEAP_START ((uint32_t) &Image$$ARM_LIB_HEAP$$ZI$$Base)
#define HEAP_END   ((uint32_t) &Image$$ARM_LIB_HEAP$$ZI$$Limit-1)
#elif defined(__GNUC__)
extern uint8_t _end; /* Symbol defined in the linker script */
extern uint32_t _Min_Heap_Size; /* Symbol defined in the linker script */
#define HEAP_START    ((uint32_t) &_end)
#define HEAP_END      ((uint32_t) &_end + (uint32_t) &_Min_Heap_Size - 1)
#endif
void MPU_Config(void)
{

  MPU_Region_InitTypeDef default_config = {0};
  MPU_Attributes_InitTypeDef attr_config = {0};
  uint32_t primask_bit = __get_PRIMASK();
  __disable_irq();

  /* disable the MPU */
  HAL_MPU_Disable();

  /* create an attribute configuration for the MPU */
  attr_config.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);
  attr_config.Number = MPU_ATTRIBUTES_NUMBER0;

  HAL_MPU_ConfigMemoryAttributes(&attr_config);

  /* Create a non cacheable region */
  /*Normal memory type, code execution unallowed */
  default_config.Enable = MPU_REGION_ENABLE;
  default_config.Number = MPU_REGION_NUMBER0;
  default_config.BaseAddress = __NON_CACHEABLE_SECTION_BEGIN;
  default_config.LimitAddress =  __NON_CACHEABLE_SECTION_END;
  default_config.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  default_config.AccessPermission = MPU_REGION_ALL_RW;
  default_config.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  default_config.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  HAL_MPU_ConfigRegion(&default_config);

  /* Create a non cacheable region for VENC HW/SW shared buffers */
  /*Normal memory type, code execution unallowed */
  default_config.Enable = MPU_REGION_ENABLE;
  default_config.Number = MPU_REGION_NUMBER1;
  default_config.BaseAddress = HEAP_START;
  default_config.LimitAddress = HEAP_END;
  default_config.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  default_config.AccessPermission = MPU_REGION_ALL_RW;
  default_config.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  default_config.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  HAL_MPU_ConfigRegion(&default_config);

  /* enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

  /* Exit critical section to lock the system and avoid any issue around MPU mechanisme */
  __set_PRIMASK(primask_bit);
}

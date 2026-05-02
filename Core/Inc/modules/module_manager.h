#ifndef MODULES_MODULE_MANAGER_H
#define MODULES_MODULE_MANAGER_H

#include "bsp_module.h"

typedef enum {
    MODULE_MGR_STATE_IDLE = 0,
    MODULE_MGR_STATE_DETECTING,
    MODULE_MGR_STATE_ACTIVE,
} module_mgr_state_t;

void module_manager_init(void);
void module_manager_task(void *arg);
bsp_module_id_t module_manager_get_active(void);
void module_manager_power_enable(bool on);

#endif /* MODULES_MODULE_MANAGER_H */

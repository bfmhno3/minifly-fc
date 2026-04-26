#include "comm/comm_stack.h"

#include "comm/radiolink.h"
#include "comm/usblink.h"
#include "comm/atkp.h"
#include "comm/commander.h"

static bool is_init = false;

void comm_stack_init(void)
{
	if (is_init) {
		return;
	}

	radiolink_init();
	usblink_init();
	atkp_init();
	commander_init();

	is_init = true;
}

bool comm_stack_test(void)
{
	return is_init;
}

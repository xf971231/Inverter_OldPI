#include <driver/dac/dac8554.h>
#include <driver/pwm/pwm.h>

#include <others/relay_and_protect.h>
#include <setup.h>
#include "sensor.h"
#include "others/keys.h"

#include "algorithm/Controller/PI/pid.h"
#include "algorithm/Controller/PR/pr.h"
#include "algorithm/PLL/pll_sogi.h"
#include "algorithm/control_para.h"
#include "algorithm/Controller/RC/crc/crc.h"
#include "driver/ad7606/ad7606.h"

#include "math.h"

void InitXintf(void);

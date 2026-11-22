#include "drive/drive.h"

#include "blocks/constant_current.h"
#include "blocks/continuous_pmsm.h"
#include "blocks/current_feedback.h"
#include "blocks/current_pi.h"
#include "blocks/inverter.h"
#include "blocks/speed_control.h"
#include "blocks/svpwm.h"

namespace pmsm {

Drive::Factory Drive::find_factory(std::string_view type) {
#define PMSM_BLOCK_TYPES(X) \
    X("pmsm", ContinuousPmsm) \
    X("speed_control", SpeedController) \
    X("current_feedback", Feedback) \
    X("current_pi", CurrentController) \
    X("svpwm", Pwm) \
    X("inverter", Inverter) \
    X("constant_current", ConstantCurrent)

#define PMSM_FACTORY(name, block) if (type == name) return &add<drive_blocks::block>;
    PMSM_BLOCK_TYPES(PMSM_FACTORY)
#undef PMSM_FACTORY
#undef PMSM_BLOCK_TYPES
    return nullptr;
}

}  // namespace pmsm

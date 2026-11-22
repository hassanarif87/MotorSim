#include "src/drive/drive.h"

#include "src/blocks/constant_current.h"
#include "src/blocks/constant_torque.h"
#include "src/blocks/continuous_pmsm.h"
#include "src/blocks/current_feedback.h"
#include "src/blocks/current_pi.h"
#include "src/blocks/inverter.h"
#include "src/blocks/simplefoc.h"
#include "src/blocks/speed_control.h"
#include "src/blocks/svpwm.h"

namespace pmsm {

Drive::Factory Drive::find_factory(std::string_view type) {
#define PMSM_BLOCK_TYPES(X) \
    X("pmsm", ContinuousPmsm) \
    X("speed_control", SpeedController) \
    X("current_feedback", Feedback) \
    X("current_pi", CurrentController) \
    X("simplefoc_velocity", SimpleFocVelocity) \
    X("simplefoc_current", SimpleFocCurrent) \
    X("svpwm", Pwm) \
    X("inverter", Inverter) \
    X("constant_current", ConstantCurrent) \
    X("ConstantTorque", ConstantTorque)

#define PMSM_FACTORY(name, block) if (type == name) return &add<drive_blocks::block>;
    PMSM_BLOCK_TYPES(PMSM_FACTORY)
#undef PMSM_FACTORY
#undef PMSM_BLOCK_TYPES
    return nullptr;
}

}  // namespace pmsm

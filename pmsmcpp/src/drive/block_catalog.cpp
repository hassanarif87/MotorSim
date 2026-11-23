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
    X(ContinuousPmsm) \
    X(SpeedController) \
    X(Feedback) \
    X(CurrentController) \
    X(SimpleFocVelocity) \
    X(SimpleFocCurrent) \
    X(Pwm) \
    X(Inverter) \
    X(NonIdealInverter) \
    X(DeadTimeCompensation) \
    X(ConstantCurrent) \
    X(ConstantTorque)

#define PMSM_FACTORY(block) if (type == #block) return &add<drive_blocks::block>;
    PMSM_BLOCK_TYPES(PMSM_FACTORY)
#undef PMSM_FACTORY
#undef PMSM_BLOCK_TYPES
    return nullptr;
}

}  // namespace pmsm

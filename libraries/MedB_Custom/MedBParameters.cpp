#include "MedBParameters.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo MedBParameters::var_info[] = {

    // @Param: FS_NEUTRAL
    // @DisplayName: Radio failsafe neutral output
    // @Description: When enabled, on radio failsafe write 1500us PWM to RCOUT channels 0,1,3,4
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("FS_NEUTRAL", 1, MedBParameters, m_fs_neutral, 0),

    AP_GROUPEND
};


MedBParameters::MedBParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}

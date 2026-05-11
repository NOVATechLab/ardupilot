#pragma once

#include <AP_Param/AP_Param.h>

class MedBParameters
{
public:

    MedBParameters();

    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8 m_fs_neutral;
};

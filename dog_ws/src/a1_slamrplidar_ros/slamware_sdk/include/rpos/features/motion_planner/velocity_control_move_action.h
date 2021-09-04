/**
* sweep_move_action.h
* Sweep Move Action
*
* Created By Tony Huang @ 2015-1-8
* Copyright (c) 2015 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include "../motion_planner/move_action.h"
#include "../location_provider/map.h"

namespace rpos { namespace actions {

    namespace detail {
        class VelocityControlMoveActionImpl;
    }

    class RPOS_CORE_API VelocityControlMoveAction : public MoveAction {
    public:
        typedef detail::VelocityControlMoveActionImpl impl_t;

        RPOS_OBJECT_CTORS_WITH_BASE(VelocityControlMoveAction, MoveAction);
        VelocityControlMoveAction(boost::shared_ptr<impl_t> impl);
        ~VelocityControlMoveAction();

    public:
        void setVelocity(float vx, float vy, float omega);
    };

} }

namespace rpos { namespace core { namespace detail {

    template <>
    struct RPOS_CORE_API action_caster < actions::VelocityControlMoveAction >
    {
        static actions::VelocityControlMoveAction cast(Action&);
    };

} } }

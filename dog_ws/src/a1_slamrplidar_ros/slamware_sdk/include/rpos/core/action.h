/**
* action.h
* Action is a robot operation
*
* Created By Tony Huang @ 2014-5-22
* Copyright (c) 2014~2017 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/object_handle.h>
#include <rpos/system/types.h>

namespace rpos { namespace core {

	typedef rpos::system::types::_u32 ActionID;

    /**
    * The status of an action
    */
	enum ActionStatus {
        // The action is created but not started
		ActionStatusWaitingForStart,

        // The action is running
		ActionStatusRunning,

        // The action is finished successfully
		ActionStatusFinished,

        // The action is paused
		ActionStatusPaused,

        // The action is stopped
		ActionStatusStopped,

        // The action is under error
		ActionStatusError
	};

	class Action;

	namespace detail {
		class ActionImpl;

		template<class ActionT>
		struct action_caster {
			static ActionT cast(Action&);
		};
	}

    /**
    * Actions are robots' movement jobs
    */
	class RPOS_CORE_API Action : public rpos::system::ObjectHandle<Action, detail::ActionImpl> {
	public:
		RPOS_OBJECT_CTORS(Action);
		~Action();

        operator bool() const;

	public:
        /**
        * Get the status of the action
        */
		ActionStatus getStatus();

        /**
        * NOT_IMPLEMENTED, get the progress of the action (if available)
        */
		double getProgress();

        /**
        * Abrot this action
        */
		void cancel();

        /**
        * Synchronizely wait the finish of the action
        */
		ActionStatus waitUntilDone();

        /**
        * OBSOLETE, check if this action is empty (please use bool(someAction) instead)
        */
        bool isEmpty() const;

        /**
        * Get the id of the action
        */
        rpos::core::ActionID getActionId();

        /**
        * Get the name to the action
        */
        std::string getActionName();

        /**
        * Get the reason that causes action failure
        */
        std::string getReason();

		template<class ActionT>
		ActionT cast() {
			return detail::action_caster<ActionT>::cast(*this);
		}

	private:
		template<class ActionT>
		friend struct detail::action_caster;
	};

} }

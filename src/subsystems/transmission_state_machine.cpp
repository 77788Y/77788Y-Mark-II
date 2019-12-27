#include "subsystems/transmission.hpp"
#include "state_machine.hpp"

namespace transmission {

  // state machine
    StateMachine<StateType, BehaviorType> state_machine(StateType::LOCKED, std::vector<State<StateType, BehaviorType>>({
      State<StateType, BehaviorType>(StateType::LOCKED),
      State<StateType, BehaviorType>(StateType::WAITING_FOR_LOCK),
      State<StateType, BehaviorType>(StateType::WAITING_FOR_UNLOCK),
      State<StateType, BehaviorType>(StateType::RETRACTING),
      State<StateType, BehaviorType>(StateType::DEPOSITED),
      State<StateType, BehaviorType>(StateType::DEPOSITING_FAST)
    }));
}
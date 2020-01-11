#include "state_machine.hpp"


// behavior
template <class T>
AbstractBehavior<T>::AbstractBehavior(T identifier) : m_identifier(identifier) {}

template <class T>
void AbstractBehavior<T>::update() {}


// state machine
template <typename TState, typename TBehavior>
StateMachine<TState, TBehavior>::StateMachine(TState entry_state): m_current_state(entry_state), m_desired_state(entry_state) {}

template <typename TState, typename TBehavior>
StateMachine<TState, TBehavior>::StateMachine(TState entry_state, std::vector<State<TState, TBehavior>> states): m_current_state(entry_state), m_desired_state(entry_state) {
  for (auto i_state = states.begin(); i_state != states.end(); ++i_state) {
    m_map.insert((*i_state).identifier, *i_state);
  }
}

template <typename TState, typename TBehavior>
void StateMachine<TState, TBehavior>::change_state(TState state) {

  m_desired_state = state;
}
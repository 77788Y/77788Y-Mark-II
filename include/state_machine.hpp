#if FALSE

#include <memory>
#include <functional>
#include <vector>
#include <map>


/**
 * A behavior that a state will possess.
 * Should be extended for each behavior in a machine.
 * 
 * \tparam T
 *         An enum describing all behaviors of a machine
 */
template <class T>
class AbstractBehavior {

public:

  AbstractBehavior(T identifier);

  virtual ~AbstractBehavior();

  /**
   * Identifies the behavior.
   * Each behavior must have an identifier.
   * Each identifier should only be used in a single behavior.
   */
  const T m_identifier;

  /**
   * Update the behavior.
   * Should be run at a steady frequency.
   * 50ms intervals or shorter are suggested.
   */
  virtual void update() = 0;
};


/**
 * A state the the state machine may be in.
 * 
 * \tparam TState
 *         An enum describing all states of a machine
 * \tparam TBehavior
 *         An enum describing all behaviors of a machine
 */
template <typename TState, typename TBehavior>
struct State {

  /**
   * Identifies the state.
   * Each state must have an identifier.
   * Each identifier should only be used in a single class.
   */
  const TState m_identifier;

  /**
   * The behavior taht will run when the machine is in the state.
   * Each state must have a behavior.
   * Behaviors may be reused across states.
   */
  std::shared_ptr<AbstractBehavior<TBehavior>> mp_behavior;

  /**
   * A function that is run as soon as the state is entered.
   */
  std::function<void()> m_enter;

  /**
   * The function that defines an exit from the state.
   * This function is triggerred EVERY machine update, NOT after an event.
   * This function should change the state (and run any exit functions) based on the desired state if conditions are meant.
   * 
   * \param T
   *        The desired state
   */
  std::function<void(TState)> m_exit;

  /**
   * A constructor for a State.
   * 
   * \param identifier
   *        The identifier for the state
   * \param behavior
   *        A pointer to the behavior of the state
   * \param enter
   *        The entry function of the state
   * \param exit
   *        The exit-check function of the state
   */
  State(TState identifier, std::shared_ptr<AbstractBehavior<TBehavior>> behavior = nullptr, std::function<void()> enter = [] {}, std::function<void(TState)> exit = [] (TState) {}): 
    m_identifier(identifier), mp_behavior(behavior), m_enter(enter), m_exit(exit) {}
};


/**
 * An instance of a state machine.
 * 
 * \tparam TState
 *         An enum describing all states of the machine
 * \tparam TBehavior
 *         An enum describing all behaviors of the machine
 */
template <typename TState, typename TBehavior>
class StateMachine {

public:


  /**
   * A constructor that simply defines the entry state.
   * 
   * \param entry_state
   *        The state at which the machine should start
   */
  StateMachine(TState entry_state);

  /**
   * A constructor that defines the entry state and the machine's State objects.
   * 
   * \param entry_state
   *        The state at which the machine should start
   * \param states
   *        The `State`s to initialize the machine with
   */
  StateMachine(TState entry_state, std::vector<State<TState, TBehavior>> states);

  /**
   * Set the desired state of the machine.
   * Depending on the `m_exit` function of the current state, the machine may not instantly switch states.
   * 
   * \param state
   *        The new desired state
   */
  void change_state(TState state);

  /**
   * Get the current state of the machine.
   * 
   * \return The current state's identifier
   */
  TState get_state();

  /**
   * Get the current behavior of the machine.
   * 
   * \return a pointer to the curret behavior
   */
  TBehavior get_behavior();

  /**
   * Add a state to the machine.
   * 
   * \param state
   *        The state to add
   * 
   * \return True if the state was addedd successfully, false otherwise
   */
  bool add_state(State<TState, TBehavior> state);

  /**
   * Remove a state from the machine.
   * 
   * \param identifier
   *        The identifier of the state that should be removed
   */
  void remove_state(TState identifier);


  /**
   * Update the machine.
   * This will update the current state's exit function and then its behavior.
   * If the active state is changed, the new state will be fully updated as soon as its enter function is finished (its exit function will run, then its behavior)
   */
  void update();


private:

  /**
   * A map that maps identifiers to states in the machine.
   */
  std::map<TState, State<TState, TBehavior>> m_map;

  /**
   * A pointer to the current state of the machine.
   */
  std::shared_ptr<State<TState, TBehavior>> m_current_state;

  /**
   * The identifier of the desired state of the machine.
   */
  TState m_desired_state;
};
#endif

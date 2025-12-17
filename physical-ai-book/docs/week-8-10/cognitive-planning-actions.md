---
title: "Cognitive Planning & Actions"
description: "Understanding cognitive planning in robotics and how intelligent agents select and execute actions"
tags: ["Cognitive Robotics", "Planning", "AI Actions", "Robot Decision Making", "Task Planning"]
---

# Cognitive Planning & Actions

## Learning Objectives

By the end of this chapter, students will be able to:
- Explain the principles of cognitive planning in robotics
- Implement task planning algorithms for robotic systems
- Design action selection mechanisms for intelligent agents
- Analyze the relationship between perception, planning, and action execution
- Evaluate the efficiency and effectiveness of planning algorithms

## Introduction to Cognitive Planning

Cognitive planning in robotics refers to the process by which intelligent robots decide what actions to perform to achieve their goals. Unlike reactive systems that respond directly to stimuli, cognitive systems use internal reasoning to determine their behavior. This involves understanding the current state of the world, predicting the outcomes of possible actions, and selecting actions that move the system toward its goals.

Cognitive planning encompasses multiple levels of abstraction:
- **Task planning**: High-level goal decomposition
- **Motion planning**: Pathfinding and trajectory generation
- **Action planning**: Sequencing of primitive actions
- **Reactive planning**: Immediate response to unexpected events

## Planning Paradigms

### Hierarchical Task Networks (HTN)

Hierarchical Task Networks decompose complex tasks into simpler subtasks in a hierarchical structure:

```python
class HTNPlanner:
    def __init__(self):
        self.primitive_actions = {
            'move_to': self.move_to,
            'pick_up': self.pick_up,
            'place_down': self.place_down
        }
        
        self.complex_tasks = {
            'assemble_product': [
                ('move_to', 'part_a_location'),
                ('pick_up', 'part_a'),
                ('move_to', 'assembly_station'),
                ('place_down', 'part_a'),
                ('move_to', 'part_b_location'),
                ('pick_up', 'part_b'),
                ('move_to', 'assembly_station'),
                ('place_down', 'part_b'),
                ('perform_assembly', 'part_a', 'part_b')
            ]
        }
    
    def plan(self, task, state):
        if task in self.primitive_actions:
            return [task]
        elif task in self.complex_tasks:
            plan = []
            for subtask in self.complex_tasks[task]:
                subplan = self.plan(subtask[0], state)
                plan.extend(subplan)
                # Update state after each subtask
                state = self.execute_and_update_state(subtask, state)
            return plan
    
    def execute_and_update_state(self, action, state):
        # Execute action and return updated state
        return state
```

### Classical Planning (STRIPS)

Classical planning uses formal logic to represent states, actions, and goals:

```python
class STRIPSAction:
    def __init__(self, name, preconditions, effects):
        self.name = name
        self.preconditions = preconditions  # Set of facts that must be true
        self.effects = effects  # Set of facts that become true/removed

class STRIPSPlanner:
    def __init__(self, actions, initial_state, goal_state):
        self.actions = actions
        self.initial_state = initial_state
        self.goal_state = goal_state
    
    def plan(self):
        # Use algorithms like A* or Forward Search
        return self.forward_search(self.initial_state, self.goal_state)
    
    def forward_search(self, start_state, goal_state):
        # Implementation of forward search algorithm
        pass
```

### Partial Order Planning

Partial order planning maintains flexibility by not ordering actions that don't need to be ordered:

```python
class PartialOrderPlan:
    def __init__(self):
        self.steps = []
        self.ordering_constraints = []  # List of (before, after) tuples
        self.causal_links = []  # List of (step, condition, step)
    
    def add_step(self, action):
        self.steps.append(action)
    
    def add_ordering(self, before, after):
        self.ordering_constraints.append((before, after))
    
    def add_causal_link(self, producer, condition, consumer):
        self.causal_links.append((producer, condition, consumer))
```

## Action Representation and Selection

### Action Models

Actions in cognitive robotics are typically represented with:

1. **Preconditions**: Conditions that must be true for the action to be executed
2. **Effects**: Changes to the world state when the action is executed
3. **Costs**: Resource consumption or time required
4. **Qualifications**: Context-dependent constraints

```python
class Action:
    def __init__(self, name, preconditions, add_effects, del_effects, cost=1):
        self.name = name
        self.preconditions = set(preconditions)
        self.add_effects = set(add_effects)  # Facts that become true
        self.del_effects = set(del_effects)  # Facts that become false
        self.cost = cost
    
    def applicable(self, state):
        return self.preconditions.issubset(state)
    
    def apply(self, state):
        if not self.applicable(state):
            raise ValueError("Action not applicable in current state")
        
        new_state = state.copy()
        new_state.update(self.add_effects)
        new_state.difference_update(self.del_effects)
        return new_state
```

### Action Selection Mechanisms

#### Utility-Based Selection

Actions are selected based on their expected utility:

```python
class UtilityBasedPlanner:
    def __init__(self, actions, utility_function):
        self.actions = actions
        self.utility_function = utility_function
    
    def select_action(self, state, goals):
        best_action = None
        best_utility = float('-inf')
        
        for action in self.actions:
            if action.applicable(state):
                resulting_state = action.apply(state)
                utility = self.utility_function(resulting_state, goals)
                
                if utility > best_utility:
                    best_utility = utility
                    best_action = action
        
        return best_action
```

#### Goal-Directed Selection

Actions are selected based on how well they contribute to achieving goals:

```python
class GoalDirectedPlanner:
    def __init__(self, actions):
        self.actions = actions
    
    def select_action(self, state, goals):
        applicable_actions = [a for a in self.actions if a.applicable(state)]
        
        # Rank actions by goal relevance
        goal_relevance = {}
        for action in applicable_actions:
            relevance = 0
            for goal in goals:
                if goal in action.add_effects:
                    relevance += 1
                elif goal in action.del_effects:
                    relevance -= 10  # Strong penalty for removing goals
            goal_relevance[action] = relevance
        
        # Return action with highest relevance
        return max(applicable_actions, key=lambda a: goal_relevance[a])
```

## Planning Algorithms

### Forward Search (Progression Planning)

Forward search explores states by applying actions forward from the initial state:

```python
def forward_search(initial_state, goal_test, actions, max_steps=100):
    from collections import deque
    
    queue = deque([(initial_state, [])])  # (state, plan)
    visited = set()
    
    while queue:
        state, plan = queue.popleft()
        
        if goal_test(state):
            return plan
        
        if len(plan) >= max_steps:
            continue
            
        state_tuple = tuple(sorted(state))
        if state_tuple in visited:
            continue
        visited.add(state_tuple)
        
        for action in actions:
            if action.applicable(state):
                new_state = action.apply(state)
                new_plan = plan + [action]
                queue.append((new_state, new_plan))
    
    return None  # No plan found
```

### Backward Search (Regression Planning)

Backward search works from the goal state backward to the initial state:

```python
def backward_search(initial_state, goal_state, actions):
    # This is a simplified version - full implementation is more complex
    def regress(goal_state, action):
        # Find a state that could lead to goal_state through action
        # This requires inverting action effects
        pass
    
    # Implementation would involve finding actions that could achieve goal_state
    # then regressing to find states that could achieve the action's preconditions
    pass
```

### Graphplan

Graphplan builds a planning graph and searches for a solution level by level:

```python
class GraphPlan:
    def __init__(self, actions, initial_state, goal_state):
        self.actions = actions
        self.initial_state = initial_state
        self.goal_state = goal_state
        self.layers = []
    
    def build_graph(self):
        # Build planning graph layer by layer
        level = 0
        current_state = self.initial_state
        
        while True:
            # Add propositions and actions to the level
            # Check for mutex relationships
            # Stop when goal can be achieved or graph levels off
            break
    
    def extract_plan(self):
        # Extract plan from the graph
        pass
```

## Reactive Planning

### Deliberative vs. Reactive Systems

Cognitive robots often combine deliberate planning with reactive behaviors:

```python
class ReactivePlanner:
    def __init__(self, high_level_planner):
        self.high_level_planner = high_level_planner
        self.current_plan = []
        self.current_step = 0
    
    def execute_step(self, state, sensor_data):
        # Check for emergencies requiring immediate response
        if self.detect_emergency(sensor_data):
            return self.emergency_response(sensor_data)
        
        # Check if current plan needs replanning
        if not self.plan_valid(state):
            self.current_plan = self.high_level_planner.plan(state)
            self.current_step = 0
        
        # Execute next step in plan
        if self.current_step < len(self.current_plan):
            action = self.current_plan[self.current_step]
            self.current_step += 1
            return action
        else:
            return None  # Plan completed
    
    def detect_emergency(self, sensor_data):
        # Check for obstacles, system failures, etc.
        return False
    
    def emergency_response(self, sensor_data):
        # Return emergency action
        return None
```

## Multi-Level Planning

### Integration of Different Planning Levels

Robots need to coordinate planning at different levels:

```python
class MultiLevelPlanner:
    def __init__(self):
        self.task_planner = HTNPlanner()
        self.motion_planner = RRTPlanner()
        self.action_planner = UtilityBasedPlanner()
    
    def plan(self, high_level_goal, environment_map):
        # Task Planning
        subtasks = self.task_planner.plan(high_level_goal)
        
        full_plan = []
        current_location = self.get_robot_location()
        
        for subtask in subtasks:
            # Motion Planning to reach subtask location
            path = self.motion_planner.plan(current_location, subtask.location)
            
            # Action Planning for subtask execution
            actions = self.action_planner.plan(subtask.description)
            
            full_plan.extend(path)
            full_plan.extend(actions)
            
            current_location = subtask.location
        
        return full_plan
```

## Planning under Uncertainty

### Probabilistic Planning

When the environment is partially observable or actions have uncertain outcomes:

```python
class ProbabilisticAction:
    def __init__(self, name, preconditions, effects_with_probability):
        self.name = name
        self.preconditions = preconditions
        # effects_with_probability: list of (effects, probability) tuples
        self.effects_with_probability = effects_with_probability

class ProbabilisticPlanner:
    def __init__(self, actions, initial_belief_state, goal_state):
        self.actions = actions
        self.initial_belief_state = initial_belief_state
        self.goal_state = goal_state
    
    def plan(self):
        # Use algorithms like POMCP (Partially Observable Monte Carlo Planning)
        pass
```

### Contingent Planning

Contingent planning accounts for observations during execution:

```python
class ContingentPlan:
    def __init__(self):
        self.plan_tree = {}  # Maps observations to actions
    
    def execute_with_feedback(self, initial_state):
        state = initial_state
        plan_node = self.plan_tree
        
        while not self.is_terminal(plan_node):
            if self.is_action_node(plan_node):
                action = plan_node['action']
                result = self.execute_action(action, state)
                
                # Update state based on action result
                state = self.update_state(state, action, result)
                
                # Select next node based on observation
                next_observation = self.get_observation(state)
                plan_node = plan_node['branches'][next_observation]
            else:
                # Wait for observation
                observation = self.get_observation(state)
                plan_node = plan_node[observation]
```

## Planning in Dynamic Environments

### Replanning Strategies

Robots operating in dynamic environments need to replan frequently:

```python
class DynamicPlanner:
    def __init__(self, base_planner, replan_threshold=0.5):
        self.base_planner = base_planner
        self.replan_threshold = replan_threshold
        self.current_plan = None
        self.plan_age = 0
    
    def update_and_execute(self, state, environment_changes):
        # If environment changed significantly, replan
        if self.environment_changed_significantly(environment_changes):
            self.current_plan = self.base_planner.plan(state)
            self.plan_age = 0
        else:
            self.plan_age += 1
        
        # If plan is too old, replan
        if self.plan_age > 10:  # Arbitrary threshold
            self.current_plan = self.base_planner.plan(state)
            self.plan_age = 0
        
        return self.execute_next_step(state, self.current_plan)
    
    def environment_changed_significantly(self, changes):
        # Determine if changes require replanning
        return any(change.impact > self.replan_threshold for change in changes)
```

## Cognitive Architecture Integration

### Planning in Cognitive Architectures

Planning modules integrate with other cognitive modules:

```python
class CognitiveRobot:
    def __init__(self):
        # Perception module
        self.perception = PerceptionModule()
        
        # Memory module
        self.memory = MemoryModule()
        
        # Planning module
        self.planner = MultiLevelPlanner()
        
        # Execution module
        self.executor = ActionExecutor()
    
    def step(self, sensor_data, goals):
        # Perceive environment
        world_state = self.perception.process_sensors(sensor_data)
        
        # Update memory
        self.memory.update(world_state)
        
        # Plan next action
        plan = self.planner.plan(goals, world_state, self.memory.get_context())
        
        # Execute plan
        action = self.executor.select_action(plan)
        return action
```

## Performance Considerations

### Planning Efficiency

Planning algorithms need to be efficient for real-time operation:

#### Heuristic Functions

Good heuristics can dramatically improve planning speed:

```python
def heuristic_distance(state, goal):
    # Manhattan distance for grid-based planning
    return abs(state.x - goal.x) + abs(state.y - goal.y)

def heuristic_manipulation(state, goal):
    # For manipulation tasks, consider joint angles and object positions
    joint_diff = sum(abs(j1 - j2) for j1, j2 in zip(state.joint_angles, goal.joint_angles))
    object_diff = distance(state.object_pose, goal.object_pose)
    return joint_diff + object_diff
```

#### Plan Repair

Instead of replanning from scratch, modify existing plans:

```python
def repair_plan(current_plan, new_constraints):
    # Identify part of plan affected by new constraints
    affected_idx = find_affected_step(current_plan, new_constraints)
    
    # Repair from affected step onwards
    if affected_idx is not None:
        repaired_suffix = replan_from_step(current_plan[:affected_idx], new_constraints)
        return current_plan[:affected_idx] + repaired_suffix
    else:
        return current_plan  # No repair needed
```

## Real-World Applications

### Service Robotics

Planning for service robots in human environments:

```python
class ServiceRobotPlanner:
    def __init__(self):
        self.task_planner = self.setup_service_task_planning()
        self.social_awareness = SocialConstraintManager()
    
    def plan_delivery_task(self, start_location, destination, user_preferences):
        # Consider social conventions (e.g., don't block doorways)
        constraints = self.social_awareness.get_constraints(destination)
        
        # Plan path with social constraints
        path = self.plan_path_with_constraints(start_location, destination, constraints)
        
        # Plan interaction with user
        interaction = self.plan_user_interaction(user_preferences)
        
        return path + interaction
```

### Manufacturing Robotics

Industrial applications with strict timing requirements:

```python
class ManufacturingPlanner:
    def __init__(self):
        self.temporal_planner = TemporalPlanner()
        self.quality_constraints = QualityConstraintManager()
    
    def plan_assembly_task(self, components, target_product, time_limit):
        # Create temporal plan with resource constraints
        plan = self.temporal_planner.create_plan(
            actions=self.get_assembly_actions(components),
            resource_limits=self.get_robotic_resources(),
            deadline=time_limit
        )
        
        # Add quality checks to plan
        plan_with_checks = self.add_quality_checks(plan, self.quality_constraints)
        
        return plan_with_checks
```

## Challenges and Future Directions

### Scalability Challenges

Planning algorithms face challenges as problem complexity increases:

#### State Space Explosion
- Exponential growth in possible states
- Need for abstraction and approximation
- Hierarchical approaches for manageability

#### Real-Time Requirements
- Planning must complete within time constraints
- Anytime algorithms that improve over time
- Approximate solutions when optimal planning is infeasible

### Integration Challenges

#### Perception-Action Loops
- Tight integration between perception and planning
- Handling uncertainty and partial observability
- Robustness to perception errors

#### Human-Robot Interaction
- Planning that considers human behavior
- Shared autonomy systems
- Plan explanation and transparency

## Exercises

1. Implement a simple STRIPS planner for a robot that needs to navigate and manipulate objects.
2. Create a hierarchical task network for a robot housekeeping task.
3. Design a reactive planning system that can replan when obstacles are detected.
4. Implement a utility-based action selection system for a multi-goal scenario.

## Quiz

1. What does HTN stand for in planning paradigms?
   - A) Hierarchical Task Network
   - B) High-level Task Navigation
   - C) Human Task Normalization
   - D) Hardware Task Network

2. Which planning approach works backwards from a goal to the initial state?
   - A) Forward search
   - B) Backward search
   - C) Anytime planning
   - D) Reactive planning

3. What is a key characteristic of contingent planning?
   - A) Plans are fixed at the beginning
   - B) Plans account for potential observations during execution
   - C) Plans ignore environmental changes
   - D) Plans use random action selection

## Reflection

Consider how cognitive planning bridges the gap between high-level goals and low-level robot control. What are the trade-offs between optimal planning and real-time requirements in robotic systems? How might planning algorithms need to evolve to handle increasingly complex real-world tasks? What role does uncertainty play in cognitive planning, and how can robots make robust decisions despite imperfect knowledge of their environment?
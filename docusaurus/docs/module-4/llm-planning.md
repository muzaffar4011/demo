---
sidebar_position: 3
title: LLM Cognitive Planning
---

# LLM Cognitive Planning for Humanoid Robots

Large Language Models (LLMs) provide cognitive planning capabilities that enable humanoid robots to understand complex natural language commands and generate appropriate action sequences. This lesson covers the integration of LLMs for robotic task planning.

## Learning Objectives

After completing this lesson, you will be able to:
- Integrate LLMs with humanoid robot systems for cognitive planning
- Design prompts for effective robotic task planning
- Implement multimodal reasoning combining vision and language
- Handle complex, multi-step commands with LLM planning

## Introduction to LLM Cognitive Planning

LLM cognitive planning for humanoid robots involves:
- **Natural Language Understanding**: Interpreting complex commands
- **Task Decomposition**: Breaking complex tasks into executable steps
- **World Modeling**: Understanding the environment and robot capabilities
- **Action Sequencing**: Generating appropriate action plans
- **Execution Monitoring**: Adapting plans based on feedback

For humanoid robots, LLM planning must consider:
- Physical constraints and capabilities
- Environmental context
- Safety requirements
- Real-time execution constraints

## LLM Integration Architecture

### Basic LLM Integration

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import openai
import json
import time

class LLMPlanningNode(Node):
    def __init__(self):
        super().__init__('llm_planning_node')

        # Publishers and subscribers
        self.command_pub = self.create_publisher(String, '/robot_commands', 10)
        self.plan_pub = self.create_publisher(String, '/planning_output', 10)

        # Subscription for high-level commands
        self.command_sub = self.create_subscription(
            String,
            '/high_level_commands',
            self.command_callback,
            10
        )

        # Initialize OpenAI client
        # In practice, you might use local models or different APIs
        self.client = openai.OpenAI()

        # Robot capabilities and environment context
        self.robot_context = {
            "capabilities": [
                "navigation",
                "object manipulation",
                "speech",
                "gesture",
                "grasping"
            ],
            "environment": {
                "locations": ["kitchen", "living room", "bedroom", "office"],
                "objects": ["cup", "book", "phone", "keys", "bottle"],
                "constraints": {
                    "max_load": "2kg",
                    "max_height": "1.8m",
                    "safety_radius": "0.5m"
                }
            }
        }

    def command_callback(self, msg):
        """Process high-level command using LLM"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        try:
            # Generate plan using LLM
            plan = self.generate_plan_with_llm(command)
            self.publish_plan(plan)
        except Exception as e:
            self.get_logger().error(f'LLM planning error: {e}')
            self.handle_planning_error(command, str(e))

    def generate_plan_with_llm(self, command):
        """Generate execution plan using LLM"""
        system_prompt = f"""
        You are a planning assistant for a humanoid robot. The robot has the following capabilities: {self.robot_context['capabilities']}.
        The environment contains these locations: {self.robot_context['environment']['locations']} and objects: {self.robot_context['environment']['objects']}.
        The robot has these constraints: {self.robot_context['environment']['constraints']}.

        Your task is to decompose natural language commands into executable steps for the robot.
        Return a JSON object with the following structure:
        {{
            "command": "original command",
            "steps": [
                {{
                    "action": "action_type",
                    "parameters": {{"param1": "value1", ...}},
                    "description": "brief description"
                }}
            ],
            "estimated_duration": "estimated time in seconds"
        }}

        Action types: 'navigate', 'grasp', 'place', 'speak', 'gesture', 'wait', 'detect_object'
        """

        user_prompt = f"Command: {command}"

        response = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.1,
            max_tokens=500
        )

        # Parse the response
        response_text = response.choices[0].message.content

        # Extract JSON from response
        try:
            # Look for JSON in response
            start_idx = response_text.find('{')
            end_idx = response_text.rfind('}') + 1
            json_str = response_text[start_idx:end_idx]

            plan = json.loads(json_str)
            return plan
        except json.JSONDecodeError:
            self.get_logger().error(f'Could not parse LLM response as JSON: {response_text}')
            return self.generate_fallback_plan(command)

    def generate_fallback_plan(self, command):
        """Generate a simple fallback plan if LLM fails"""
        # Simple rule-based fallback
        if "bring me" in command or "get me" in command:
            obj = command.split("bring me")[-1].split("get me")[-1].strip()
            return {
                "command": command,
                "steps": [
                    {"action": "detect_object", "parameters": {"object": obj}, "description": f"Locate {obj}"},
                    {"action": "navigate", "parameters": {"target": "location_of_object"}, "description": f"Go to {obj}"},
                    {"action": "grasp", "parameters": {"object": obj}, "description": f"Pick up {obj}"},
                    {"action": "navigate", "parameters": {"target": "user_location"}, "description": "Return to user"},
                    {"action": "place", "parameters": {"location": "in_front_of_user"}, "description": "Place object for user"}
                ],
                "estimated_duration": 120
            }
        else:
            return {
                "command": command,
                "steps": [
                    {"action": "speak", "parameters": {"text": f"I don't know how to {command}"}, "description": "Express inability to perform command"}
                ],
                "estimated_duration": 5
            }

    def publish_plan(self, plan):
        """Publish the generated plan"""
        plan_msg = String()
        plan_msg.data = json.dumps(plan)
        self.plan_pub.publish(plan_msg)

        # Execute the plan
        self.execute_plan(plan)

    def execute_plan(self, plan):
        """Execute the plan steps"""
        for step in plan['steps']:
            self.get_logger().info(f'Executing: {step["description"]}')

            # Publish individual commands
            cmd_msg = String()
            cmd_msg.data = json.dumps(step)
            self.command_pub.publish(cmd_msg)

            # Simple delay - in real system, wait for completion
            time.sleep(2)

    def handle_planning_error(self, command, error_msg):
        """Handle planning errors"""
        error_response = {
            "command": command,
            "error": error_msg,
            "suggestion": "Please rephrase your command or provide more specific instructions"
        }

        error_msg = String()
        error_msg.data = json.dumps(error_response)
        self.plan_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Multimodal Reasoning with Vision

### Vision-Language Integration

```python
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MultimodalLLMPlanner:
    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()

        # Subscribe to camera feed
        self.image_sub = node.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.latest_image = None
        self.image_timestamp = None

    def image_callback(self, msg):
        """Process incoming image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            self.image_timestamp = msg.header.stamp
        except Exception as e:
            self.node.get_logger().error(f'Image conversion error: {e}')

    def generate_vision_guided_plan(self, command, image=None):
        """Generate plan with visual context"""
        if image is None:
            image = self.latest_image

        if image is not None:
            # In practice, use multimodal models that can process both text and images
            # For now, we'll describe the image content
            image_description = self.describe_image_content(image)

            system_prompt = f"""
            You are a planning assistant for a humanoid robot. The robot has the following capabilities: {self.node.robot_context['capabilities']}.
            The current visual scene shows: {image_description}
            The environment contains these locations: {self.node.robot_context['environment']['locations']} and objects: {self.node.robot_context['environment']['objects']}.

            Use the visual information to help interpret the command and generate a more accurate plan.
            """
        else:
            system_prompt = f"""
            You are a planning assistant for a humanoid robot. The robot has the following capabilities: {self.node.robot_context['capabilities']}.
            The environment contains these locations: {self.node.robot_context['environment']['locations']} and objects: {self.node.robot_context['environment']['objects']}.
            """

        user_prompt = f"Command: {command}"

        # Call LLM with multimodal context
        response = self.node.client.chat.completions.create(
            model="gpt-4-vision-preview",  # Use vision-capable model if available
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.1,
            max_tokens=500
        )

        # Process response similar to basic LLM integration
        response_text = response.choices[0].message.content
        # Extract and return plan...

        return self.parse_plan_response(response_text)

    def describe_image_content(self, image):
        """Simple image description (in practice, use computer vision models)"""
        # This is a simplified example
        # In practice, use object detection, scene understanding, etc.
        height, width, channels = image.shape

        # Example: detect if image is indoor/outdoor, approximate number of objects, etc.
        return f"Image is {width}x{height} pixels with {channels} channels. Scene appears to be indoor with multiple objects visible."
```

## Context-Aware Planning

### Maintaining Planning Context

```python
class ContextAwarePlanner:
    def __init__(self, node):
        self.node = node
        self.context_history = []
        self.current_plan = None
        self.execution_state = "idle"

    def update_context(self, observation):
        """Update planning context with new observations"""
        context_entry = {
            "timestamp": self.node.get_clock().now().to_msg(),
            "observation": observation,
            "robot_state": self.get_robot_state(),
            "environment_state": self.get_environment_state()
        }

        self.context_history.append(context_entry)

        # Keep only recent context (last 10 entries)
        if len(self.context_history) > 10:
            self.context_history = self.context_history[-10:]

    def get_robot_state(self):
        """Get current robot state"""
        # In practice, this would come from robot state publisher
        return {
            "location": "current_location",
            "battery": 85,
            "carrying": None,
            "executing_task": bool(self.current_plan)
        }

    def get_environment_state(self):
        """Get current environment state"""
        # In practice, this would come from perception system
        return {
            "known_objects": ["cup", "book"],
            "free_space": True,
            "obstacles": []
        }

    def adapt_plan_to_context(self, original_plan, context):
        """Adapt plan based on current context"""
        # Check if original plan is still valid given current context
        if not self.is_plan_still_valid(original_plan, context):
            # Regenerate plan with new context
            return self.regenerate_plan_with_context(original_plan, context)

        return original_plan

    def is_plan_still_valid(self, plan, context):
        """Check if plan is still valid given current context"""
        # Check if environment has changed significantly
        # Check if robot state allows plan execution
        # Check if goal is still relevant

        # Simple example: if carrying object changed
        if context["robot_state"]["carrying"] != plan.get("required_carrying_state"):
            return False

        return True
```

## Advanced Planning Strategies

### Hierarchical Task Planning

```python
class HierarchicalPlanner:
    def __init__(self, node):
        self.node = node
        self.task_hierarchy = {}

    def create_hierarchical_plan(self, high_level_command):
        """Create hierarchical plan with multiple levels of abstraction"""
        # High-level plan
        high_level_plan = self.generate_high_level_plan(high_level_command)

        # Decompose into subtasks
        detailed_plan = self.decompose_into_subtasks(high_level_plan)

        # Add execution details
        executable_plan = self.add_execution_details(detailed_plan)

        return executable_plan

    def generate_high_level_plan(self, command):
        """Generate high-level task decomposition"""
        system_prompt = """
        Decompose the given command into high-level tasks that can be executed by a humanoid robot.
        Each task should be meaningful and achievable.
        Return in JSON format with 'tasks' array.
        """

        user_prompt = f"Decompose this command: {command}"

        response = self.node.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.1,
            max_tokens=300
        )

        return json.loads(response.choices[0].message.content)

    def decompose_into_subtasks(self, high_level_plan):
        """Decompose high-level tasks into executable subtasks"""
        detailed_plan = {"tasks": []}

        for task in high_level_plan["tasks"]:
            subtasks = self.generate_subtasks_for_task(task)
            detailed_plan["tasks"].append({
                "task": task,
                "subtasks": subtasks
            })

        return detailed_plan

    def generate_subtasks_for_task(self, task):
        """Generate executable subtasks for a high-level task"""
        # Use LLM to generate subtasks
        system_prompt = """
        For the given high-level task, generate specific executable subtasks that a humanoid robot can perform.
        Each subtask should be simple and specific.
        """

        user_prompt = f"Generate subtasks for: {task}"

        response = self.node.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.1,
            max_tokens=300
        )

        return json.loads(response.choices[0].message.content)
```

## Safety and Validation

### Plan Validation Framework

```python
class PlanValidator:
    def __init__(self, node):
        self.node = node

    def validate_plan(self, plan):
        """Validate plan for safety and feasibility"""
        validation_results = {
            "is_safe": True,
            "is_feasible": True,
            "issues": [],
            "suggestions": []
        }

        # Check safety constraints
        safety_check = self.check_safety_constraints(plan)
        if not safety_check["passed"]:
            validation_results["is_safe"] = False
            validation_results["issues"].extend(safety_check["issues"])
            validation_results["suggestions"].extend(safety_check["suggestions"])

        # Check feasibility
        feasibility_check = self.check_feasibility(plan)
        if not feasibility_check["passed"]:
            validation_results["is_feasible"] = False
            validation_results["issues"].extend(feasibility_check["issues"])
            validation_results["suggestions"].extend(feasibility_check["suggestions"])

        return validation_results

    def check_safety_constraints(self, plan):
        """Check if plan violates safety constraints"""
        issues = []
        suggestions = []

        for step in plan.get("steps", []):
            action = step.get("action", "")
            params = step.get("parameters", {})

            # Check navigation safety
            if action == "navigate":
                target = params.get("target", "")
                if self.is_unsafe_location(target):
                    issues.append(f"Navigation to {target} may be unsafe")
                    suggestions.append(f"Find alternative route to {target}")

            # Check manipulation safety
            elif action == "grasp":
                obj = params.get("object", "")
                if self.is_hazardous_object(obj):
                    issues.append(f"Grasping {obj} may be hazardous")
                    suggestions.append(f"Avoid grasping {obj} or use protective measures")

        return {
            "passed": len(issues) == 0,
            "issues": issues,
            "suggestions": suggestions
        }

    def is_unsafe_location(self, location):
        """Check if location is unsafe"""
        # In practice, this would check against known unsafe locations
        unsafe_locations = ["construction_zone", "restricted_area"]
        return location in unsafe_locations

    def is_hazardous_object(self, obj):
        """Check if object is hazardous"""
        # In practice, this would check against object database
        hazardous_objects = ["knife", "chemical", "hot_item"]
        return obj.lower() in [hazard.lower() for hazard in hazardous_objects]

    def check_feasibility(self, plan):
        """Check if plan is physically feasible"""
        issues = []
        suggestions = []

        # Check if robot can perform required actions
        for step in plan.get("steps", []):
            action = step.get("action", "")
            if not self.is_action_feasible(action):
                issues.append(f"Action {action} is not feasible for this robot")
                suggestions.append(f"Replace {action} with alternative action")

        return {
            "passed": len(issues) == 0,
            "issues": issues,
            "suggestions": suggestions
        }

    def is_action_feasible(self, action):
        """Check if action is feasible for robot"""
        feasible_actions = [
            "navigate", "grasp", "place", "speak", "gesture",
            "detect_object", "wait", "approach", "avoid"
        ]
        return action in feasible_actions
```

## Performance Optimization

### Caching and Pre-planning

```python
import hashlib
from functools import lru_cache

class OptimizedLLMPlanner:
    def __init__(self, node):
        self.node = node
        self.plan_cache = {}
        self.max_cache_size = 100

    @lru_cache(maxsize=100)
    def get_cached_plan(self, command_hash):
        """Get cached plan for command hash"""
        # This is called by the decorator
        pass

    def generate_plan_with_caching(self, command):
        """Generate plan with caching to improve performance"""
        # Create hash of command for caching
        command_hash = hashlib.md5(command.encode()).hexdigest()

        # Check cache first
        if command_hash in self.plan_cache:
            self.node.get_logger().info("Using cached plan")
            return self.plan_cache[command_hash]

        # Generate new plan
        plan = self.generate_plan_with_llm(command)

        # Cache the plan
        if len(self.plan_cache) >= self.max_cache_size:
            # Remove oldest entry
            oldest_key = next(iter(self.plan_cache))
            del self.plan_cache[oldest_key]

        self.plan_cache[command_hash] = plan

        return plan
```

## Exercise

Create an LLM cognitive planning system that:
1. Integrates with OpenAI or similar LLM service
2. Generates detailed execution plans for complex commands
3. Incorporates visual context from camera feeds
4. Implements plan validation for safety and feasibility
5. Handles plan adaptation when context changes

## Summary

LLM cognitive planning provides sophisticated task decomposition and execution planning for humanoid robots. By combining natural language understanding with contextual awareness, robots can execute complex, multi-step commands. Proper validation and safety checking ensure reliable operation. The next lesson will cover the capstone project integrating all modules.
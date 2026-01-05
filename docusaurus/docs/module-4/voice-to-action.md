---
sidebar_position: 2
title: Voice-to-Action
---

# Voice-to-Action Systems for Humanoid Robots

Voice-to-action systems enable humanoid robots to understand spoken commands and translate them into robotic actions. This technology provides a natural interface for human-robot interaction.

## Learning Objectives

After completing this lesson, you will be able to:
- Implement speech recognition systems for humanoid robots
- Process natural language commands for robotic execution
- Handle speech recognition errors and fallback mechanisms
- Integrate voice commands with robotic action planning

## Introduction to Voice-to-Action Systems

Voice-to-action systems for humanoid robots involve:
- **Speech Recognition**: Converting spoken language to text
- **Natural Language Understanding**: Interpreting the meaning of commands
- **Action Planning**: Converting commands into robotic actions
- **Execution Monitoring**: Ensuring actions are performed correctly

For humanoid robots, voice-to-action systems must handle:
- Noisy environments
- Multiple speakers
- Complex command structures
- Real-time processing requirements

## Speech Recognition with OpenAI Whisper

### Basic Whisper Integration

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import openai
import numpy as np
import pyaudio
import wave
import threading
import queue

class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')

        # Publishers and subscribers
        self.command_pub = self.create_publisher(String, '/robot_commands', 10)
        self.status_pub = self.create_publisher(String, '/voice_status', 10)

        # Audio recording parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.record_seconds = 5

        # Initialize audio stream
        self.audio = pyaudio.PyAudio()
        self.audio_queue = queue.Queue()

        # Start audio recording thread
        self.recording = True
        self.recording_thread = threading.Thread(target=self.record_audio)
        self.recording_thread.start()

        # Timer for processing audio
        self.timer = self.create_timer(1.0, self.process_audio)

    def record_audio(self):
        """Record audio from microphone"""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        while self.recording:
            data = stream.read(self.chunk)
            self.audio_queue.put(data)

        stream.stop_stream()
        stream.close()

    def process_audio(self):
        """Process recorded audio and convert to text"""
        if not self.audio_queue.empty():
            # Collect audio frames
            frames = []
            while not self.audio_queue.empty():
                frames.append(self.audio_queue.get())

            # Save to temporary WAV file for Whisper processing
            wf = wave.open('temp_audio.wav', 'wb')
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))
            wf.close()

            # Process with Whisper
            try:
                with open('temp_audio.wav', 'rb') as audio_file:
                    transcript = openai.Audio.transcribe("whisper-1", audio_file)
                    command_text = transcript.text

                self.get_logger().info(f'Recognized: {command_text}')
                self.process_command(command_text)
            except Exception as e:
                self.get_logger().error(f'Speech recognition error: {e}')
                self.handle_recognition_error()

    def process_command(self, command_text):
        """Process the recognized command text"""
        # Publish the recognized command
        cmd_msg = String()
        cmd_msg.data = command_text
        self.command_pub.publish(cmd_msg)

        # Determine action based on command
        action = self.parse_command(command_text)
        if action:
            self.execute_action(action)

    def parse_command(self, command_text):
        """Parse natural language command into robotic action"""
        command_text = command_text.lower().strip()

        # Simple command parsing (in practice, use more sophisticated NLU)
        if 'walk to' in command_text or 'go to' in command_text:
            # Extract destination
            destination = command_text.split('to')[-1].strip()
            return {'action': 'navigate', 'target': destination}

        elif 'pick up' in command_text or 'grasp' in command_text:
            # Extract object
            obj = command_text.split('up')[-1].strip() if 'up' in command_text else command_text.split('grasp')[-1].strip()
            return {'action': 'grasp', 'object': obj}

        elif 'wave' in command_text:
            return {'action': 'gesture', 'type': 'wave'}

        elif 'sit' in command_text:
            return {'action': 'posture', 'type': 'sit'}

        elif 'stand' in command_text:
            return {'action': 'posture', 'type': 'stand'}

        else:
            return None

    def execute_action(self, action):
        """Execute the parsed action on the humanoid robot"""
        self.get_logger().info(f'Executing action: {action}')

        # In a real implementation, this would send commands to the robot
        # For now, just log the action
        action_msg = String()
        action_msg.data = f"EXECUTE: {action}"
        self.status_pub.publish(action_msg)

    def handle_recognition_error(self):
        """Handle speech recognition errors with fallback mechanisms"""
        error_msg = String()
        error_msg.data = "SPEECH_RECOGNITION_ERROR"
        self.status_pub.publish(error_msg)

        # In a real system, you might:
        # - Ask for repetition: "I didn't understand, could you repeat that?"
        # - Use alternative input methods
        # - Implement error recovery strategies

    def destroy_node(self):
        """Clean up resources"""
        self.recording = False
        if self.recording_thread.is_alive():
            self.recording_thread.join()

        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceToActionNode()

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

## Advanced Speech Recognition Techniques

### Keyword Spotting for Wake Word Detection

```python
#!/usr/bin/env python3

import numpy as np
from scipy import signal
import threading
import queue

class KeywordSpotter:
    def __init__(self, wake_word="robot"):
        self.wake_word = wake_word
        self.audio_buffer = queue.Queue()
        self.detected = False

    def detect_wake_word(self, audio_data):
        """Simple keyword spotting algorithm"""
        # In practice, use more sophisticated methods like:
        # - Hidden Markov Models
        # - Deep neural networks
        # - Pre-trained keyword spotting models

        # This is a simplified example
        # Convert audio to text and check for wake word
        # In real implementation, use audio features directly

        return self.wake_word.lower() in audio_data.lower()
```

### Real-time Audio Processing

```python
import pyaudio
import webrtcvad
import collections

class RealTimeAudioProcessor:
    def __init__(self):
        # Initialize WebRTC VAD (Voice Activity Detection)
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)  # Aggressiveness mode: 0-3

        # Audio parameters
        self.sample_rate = 16000
        self.frame_duration = 30  # ms
        self.frame_size = int(self.sample_rate * self.frame_duration / 1000)

        # Ring buffer for audio
        self.ring_buffer = collections.deque(maxlen=30)

    def is_speech_detected(self, audio_frame):
        """Detect if speech is present in audio frame"""
        try:
            return self.vad.is_speech(audio_frame, self.sample_rate)
        except:
            return False
```

## Natural Language Understanding for Robotics

### Command Parsing with Context

```python
class NaturalLanguageProcessor:
    def __init__(self):
        self.robot_state = {
            'location': 'start',
            'carrying': None,
            'battery': 100
        }

        # Define command patterns
        self.command_patterns = {
            'navigation': [
                r'go to (.+)',
                r'walk to (.+)',
                r'move to (.+)',
                r'navigate to (.+)'
            ],
            'manipulation': [
                r'pick up (.+)',
                r'grasp (.+)',
                r'get (.+)',
                r'take (.+)'
            ],
            'interaction': [
                r'wave',
                r'nod',
                r'greet',
                r'say hello'
            ]
        }

    def parse_command_with_context(self, command_text):
        """Parse command considering current robot state"""
        import re

        # Update context based on command
        for action_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, command_text, re.IGNORECASE)
                if match:
                    obj = match.group(1) if len(match.groups()) > 0 else None

                    return {
                        'action_type': action_type,
                        'object': obj,
                        'original_command': command_text,
                        'context': self.robot_state.copy()
                    }

        return None
```

## Robust Voice Command Handling

### Error Handling and Fallback Strategies

```python
class RobustVoiceHandler:
    def __init__(self):
        self.confidence_threshold = 0.7
        self.retries = 3
        self.context_history = []

    def handle_command_with_confidence(self, command_text, confidence_score):
        """Handle command based on confidence level"""
        if confidence_score >= self.confidence_threshold:
            return self.process_command(command_text)
        else:
            return self.request_clarification(command_text, confidence_score)

    def request_clarification(self, command_text, confidence_score):
        """Request user clarification for low-confidence commands"""
        # Publish request for clarification
        clarification_msg = f"Could you please repeat that? I understood: '{command_text}' with confidence {confidence_score:.2f}"

        # In a real system, this would be spoken to the user
        print(clarification_msg)

        return {'status': 'clarification_requested', 'message': clarification_msg}
```

## Integration with ROS 2

### Voice Command Interface

```yaml
# ROS 2 interface definition for voice commands
# voice_command_interface.yaml
voice_command_interface:
  ros__parameters:
    speech_recognition:
      model: "whisper-large-v2"
      language: "en"
      sample_rate: 16000
      chunk_size: 1024

    nlu:
      confidence_threshold: 0.7
      max_command_length: 100
      timeout_seconds: 5

    fallback:
      enable: true
      max_retries: 3
      alternative_input_topic: "/alternative_input"
```

### Action Server for Voice Commands

```python
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse
import rclpy
from rclpy.node import Node

class VoiceCommandActionServer(Node):
    def __init__(self):
        super().__init__('voice_command_action_server')
        self._action_server = ActionServer(
            self,
            VoiceCommand,  # Custom action message
            'execute_voice_command',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        """Execute voice command with feedback"""
        self.get_logger().info('Executing voice command...')

        command = goal_handle.request.command

        # Process command
        result = self.process_voice_command(command)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result
```

## Performance Optimization

### Audio Processing Pipeline

```python
import asyncio
import concurrent.futures

class OptimizedAudioProcessor:
    def __init__(self):
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)
        self.audio_queue = asyncio.Queue()

    async def process_audio_stream(self):
        """Asynchronously process audio stream"""
        while True:
            audio_chunk = await self.audio_queue.get()

            # Process in separate thread to avoid blocking
            future = self.executor.submit(self.process_chunk, audio_chunk)

            # Non-blocking result retrieval
            result = await asyncio.get_event_loop().run_in_executor(None, future.result)

            if result:
                await self.handle_recognized_command(result)
```

## Exercise

Create a voice-to-action system that:
1. Records audio from a microphone
2. Uses OpenAI Whisper for speech recognition
3. Parses commands for navigation and simple manipulation tasks
4. Implements error handling and fallback mechanisms
5. Integrates with ROS 2 for robot command execution

## Summary

Voice-to-action systems provide natural interfaces for humanoid robots, enabling communication through spoken language. Proper implementation requires robust speech recognition, natural language understanding, and error handling. The next lesson will cover LLM cognitive planning for more sophisticated command interpretation.
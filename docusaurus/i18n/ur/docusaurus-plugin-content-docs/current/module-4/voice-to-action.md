---
sidebar_position: 2
title: Voice-to-Action
---

# ہیومینوئڈ روبوٹس کے لیے Voice-to-Action Systems

Voice-to-action systems ہیومینوئڈ روبوٹس کو spoken commands سمجھنے اور انہیں robotic actions میں تبدیل کرنے کے قابل بناتے ہیں۔ یہ ٹیکنالوجی human-robot interaction کے لیے ایک natural interface فراہم کرتی ہے۔

## سیکھنے کے مقاصد

اس سبق کو مکمل کرنے کے بعد، آپ قابل ہوں گے:
- ہیومینوئڈ روبوٹس کے لیے speech recognition systems لاگو کرنا
- Robotic execution کے لیے natural language commands process کرنا
- Speech recognition errors اور fallback mechanisms handle کرنا
- Voice commands کو robotic action planning کے ساتھ انضمام کرنا

## Voice-to-Action Systems کا تعارف

ہیومینوئڈ روبوٹس کے لیے voice-to-action systems شامل کرتے ہیں:
- **Speech Recognition**: Spoken language کو text میں تبدیل کرنا
- **Natural Language Understanding**: Commands کے معنی کی تشریح
- **Action Planning**: Commands کو robotic actions میں تبدیل کرنا
- **Execution Monitoring**: یقینی بنانا کہ actions صحیح طریقے سے انجام دیے جاتے ہیں

ہیومینوئڈ روبوٹس کے لیے، voice-to-action systems کو handle کرنا ضروری ہے:
- Noisy environments
- Multiple speakers
- پیچیدہ command structures
- ریئل-ٹائم processing requirements

## OpenAI Whisper کے ساتھ Speech Recognition

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
            
            # Convert to audio file format
            audio_data = b''.join(frames)
            
            # Use Whisper API for transcription
            # In practice, you might use local Whisper model
            try:
                # Save to temporary file
                temp_file = "temp_audio.wav"
                with wave.open(temp_file, 'wb') as wf:
                    wf.setnchannels(self.channels)
                    wf.setsampwidth(self.audio.get_sample_size(self.format))
                    wf.setframerate(self.rate)
                    wf.writeframes(audio_data)
                
                # Transcribe using Whisper
                with open(temp_file, 'rb') as audio_file:
                    transcript = openai.Audio.transcribe("whisper-1", audio_file)
                    text = transcript["text"]
                    
                    # Publish command
                    command_msg = String()
                    command_msg.data = text
                    self.command_pub.publish(command_msg)
                    
                    self.get_logger().info(f'Recognized: {text}')
            except Exception as e:
                self.get_logger().error(f'Speech recognition error: {e}')

def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceToActionNode()

    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_node.recording = False
        voice_node.recording_thread.join()
        voice_node.audio.terminate()
        voice_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Command Processing اور Action Planning

### Natural Language Command Parser

```python
class CommandParser:
    def __init__(self):
        self.action_keywords = {
            'move': 'navigation',
            'go': 'navigation',
            'walk': 'navigation',
            'pick': 'manipulation',
            'grasp': 'manipulation',
            'bring': 'manipulation',
            'speak': 'communication',
            'say': 'communication'
        }
    
    def parse_command(self, text):
        """Parse natural language command into structured action"""
        text_lower = text.lower()
        
        # Identify action type
        action_type = None
        for keyword, action in self.action_keywords.items():
            if keyword in text_lower:
                action_type = action
                break
        
        # Extract target or location
        # This is simplified - real implementation would use NLP
        target = self.extract_target(text_lower)
        
        return {
            'action_type': action_type,
            'target': target,
            'original_text': text
        }
    
    def extract_target(self, text):
        """Extract target object or location from command"""
        # Simplified extraction
        # Real implementation would use named entity recognition
        common_targets = ['cup', 'book', 'phone', 'kitchen', 'living room']
        for target in common_targets:
            if target in text:
                return target
        return None
```

## Error Handling اور Fallback Mechanisms

### Robust Command Processing

```python
class RobustCommandProcessor(Node):
    def __init__(self):
        super().__init__('robust_command_processor')
        
        self.command_sub = self.create_subscription(
            String,
            '/robot_commands',
            self.command_callback,
            10
        )
        
        self.confirmation_pub = self.create_publisher(
            String,
            '/command_confirmation',
            10
        )
        
        self.parser = CommandParser()
    
    def command_callback(self, msg):
        """Process command with error handling"""
        try:
            parsed = self.parser.parse_command(msg.data)
            
            if parsed['action_type'] is None:
                # Ask for clarification
                self.request_clarification(msg.data)
            else:
                # Confirm and execute
                self.confirm_and_execute(parsed)
                
        except Exception as e:
            self.get_logger().error(f'Command processing error: {e}')
            self.handle_error(msg.data, str(e))
    
    def request_clarification(self, command):
        """Request clarification for ambiguous commands"""
        confirmation = String()
        confirmation.data = f"Could you clarify: {command}?"
        self.confirmation_pub.publish(confirmation)
    
    def confirm_and_execute(self, parsed_command):
        """Confirm command and initiate execution"""
        confirmation = String()
        confirmation.data = f"Executing: {parsed_command['action_type']} for {parsed_command['target']}"
        self.confirmation_pub.publish(confirmation)
        # Execute command...
```

## مشق

ایک voice-to-action system لاگو کریں جو:
- Microphone سے audio record کرتا ہے
- Whisper استعمال کرتے ہوئے speech کو text میں تبدیل کرتا ہے
- Natural language commands parse کرتا ہے
- Appropriate robotic actions generate کرتا ہے
- Errors handle کرتا ہے اور clarification request کرتا ہے

## خلاصہ

Voice-to-action systems ہیومینوئڈ روبوٹس کے لیے natural human-robot interaction فراہم کرتے ہیں۔ Speech recognition، natural language understanding، اور action planning کا مناسب انضمام intelligent روبوٹس بناتا ہے جو complex voice commands کی پیروی کر سکتے ہیں۔ اگلے سبق میں، ہم cognitive planning کے لیے LLM integration کو دریافت کریں گے۔


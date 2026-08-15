import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

import sounddevice as sd
import numpy as np
import queue
import threading
import re
import json
from faster_whisper import WhisperModel
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate

device = "C270" #the ID of mic, using python3 -m sounddevice

samplerate, block_duration, channels = 16000, 0.1, 1 #check the mic specs, 16000 is standard for speech models

frames_per_block = int(samplerate * block_duration)
VOLUME_THRESHOLD, SILENCE_LIMIT = 15.0, 8

audio_queue = queue.Queue()
text_queue = queue.Queue()

whisper_model = WhisperModel("small.en", device="cuda", compute_type="float16")
intent_model = OllamaLLM(model="llama3", format="json") 

GRASPABLE_OBJECTS = ["apple", "bottle", "cup", "banana", "remote", "cell phone", 'broccoli', 'orange', 'donut', 'sandwich', 'book'] # truncated for brevity
intent_prompt = ChatPromptTemplate.from_template(
    """Map request to ONE object: {allowed_classes}\nRequest: "{command}"\nOutput JSON with key "target"."""
)
intent_chain = intent_prompt | intent_model

# --- AUDIO CALLBACKS ---
def audio_callback(indata, frames, time, status):
    audio_queue.put(indata.copy())

def recorder():
    with sd.InputStream(device=device, samplerate=samplerate, channels=channels, callback=audio_callback, blocksize=frames_per_block):
        while True: sd.sleep(100)

def transcriber():
    audio_buffer, is_speaking, silence_blocks = [], False, 0
    while True:
        block = audio_queue.get()
        volume = np.linalg.norm(block) * 10
        if volume > VOLUME_THRESHOLD:
            is_speaking, silence_blocks = True, 0
            audio_buffer.append(block)
        elif is_speaking:
            silence_blocks += 1
            audio_buffer.append(block)
            if silence_blocks > SILENCE_LIMIT:
                is_speaking = False
                audio_data = np.concatenate(audio_buffer).flatten().astype(np.float32)
                audio_buffer = []
                segments, _ = whisper_model.transcribe(audio_data, language="en", vad_filter=True, condition_on_previous_text=False)
                text = "".join([s.text for s in segments]).strip()
                clean_text = text.lower().strip(".,!?")
                if clean_text: text_queue.put(text)

# --- ROS2 BRAIN NODE ---
class LLMClient(Node):
    def __init__(self):
        super().__init__('llm_client')
        self.pub_cmd = self.create_publisher(String, '/system_command', 10)
        
        self.sub_hand = self.create_subscription(Point, '/hand_coords', self.hand_cb, 10)
        self.sub_vision = self.create_subscription(Point, '/vision_coords', self.vision_cb, 10)
        
        self.ros_data_queue = queue.Queue()
        
        # Start AI threads
        threading.Thread(target=recorder, daemon=True).start()
        threading.Thread(target=transcriber, daemon=True).start()
        
        # Start main decision loop in a separate thread so ROS can spin
        threading.Thread(target=self.main_decision_loop, daemon=True).start()

    def hand_cb(self, msg):
        self.ros_data_queue.put(("hand", (msg.x, msg.y, msg.z)))

    def vision_cb(self, msg):
        self.ros_data_queue.put(("vision", (msg.x, msg.y)))

    def main_decision_loop(self):
        print("System active. Awaiting wake word...")
        wake_words = ["hey mara", "hey maura", "mara", "hey mora", "mora", "maura", "moro", "hey moro"] 
        reposition_phrases = ["calibrate", "track my hand"]
        hand_coords = "Not Set"

        while True:
            raw_input = text_queue.get()
            user_input = re.sub(r'[^\w\s]', '', raw_input).lower().strip()
            print(f"Heard: '{user_input}'")

            active_wake = next((w for w in wake_words if w in user_input), None)

            if active_wake:
                command_clean = user_input.replace(active_wake, "").strip()
                if not command_clean:
                    command_clean = re.sub(r'[^\w\s]', '', text_queue.get()).lower().strip()
                
                # --- CALIBRATION ---
                if any(p in command_clean for p in reposition_phrases):
                    print("Action: Triggering Hand Tracking...")
                    self.pub_cmd.publish(String(data="TRACK_HAND"))
                    
                    # Blocks here until hand_cb puts data in the queue!
                    source, coords = self.ros_data_queue.get() 
                    if source == "hand":
                        hand_coords = coords
                        print(f"Lock established at {hand_coords}. What should I grab?")
                        command_clean = re.sub(r'[^\w\s]', '', text_queue.get()).lower().strip()

                # --- OBJECT EXECUTION ---
                print("Thinking...")
                raw_llm = intent_chain.invoke({"command": command_clean, "allowed_classes": ", ".join(GRASPABLE_OBJECTS)}).strip()
                try:
                    clean_json = raw_llm.replace("```json", "").replace("```", "").strip()
                    target_object = json.loads(clean_json).get("target", "unknown").lower()
                except:
                    target_object = "unknown"

                if target_object != "unknown":
                    print(f"Target parsed: '{target_object}'. Triggering Vision...")
                    self.pub_cmd.publish(String(data=f"FIND:{target_object}"))
                    
                    # Blocks here until vision_cb puts data in the queue!
                    source, coords = self.ros_data_queue.get()
                    if source == "vision" and coords[0] != -1.0: # -1.0 is our timeout flag
                        print("\n--- EXECUTION REPORT ---")
                        print(f"Hand XYZ:  {hand_coords}")
                        print(f"Object XY: {coords}")
                        print("------------------------\nSystem active...")
                    else:
                        print("Vision Timeout. Object not found.")

def main(args=None):
    rclpy.init(args=args)
    node = LLMClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
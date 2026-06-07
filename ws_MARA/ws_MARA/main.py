import sounddevice as sd # fetches raw microphone data
import numpy as np # checks how loud the room is which we will use as a baseline
import queue # creates a que of data to be passed around
import threading # allows the mic, transcriber and main logic to run in series
import re # used to strip out punctuations from voice commands 
import json # used to parse the output form ollama
from faster_whisper import WhisperModel # everything here and below are imports for whisper, langchain and our defined camera scripts
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate
from detect import run_vision_system
from handDetection import run_hand_tracking

# --- SETTINGS & BUFFERS ---
samplerate = 16000 # this value is the standard audio quality for voice recognition
block_duration = 0.1 # grabs audio in tiny 100ms chunks
channels = 1 # mono audio, meaning not stereo
frames_per_block = int(samplerate * block_duration) # calculates how many data points are in 100ms

VOLUME_THRESHOLD = 15.0 # similar to a noise gate, higher value = less responsive
SILENCE_LIMIT = 8  # How many 100ms blocks of silence before it assumes we stopped talking 

audio_queue = queue.Queue() # this holds audio data
text_queue = queue.Queue() # this holds translated text sentences

whisper_model = WhisperModel("small.en", device="cpu", compute_type="int8")

# --- AUDIO ENGINE ---
def audio_callback(indata, frames, time, status): #This copies whatever the mic just heard and drops it on the que variable
    audio_queue.put(indata.copy())

def recorder(): # opens our webcam and runs the call back above
    with sd.InputStream(device=0, samplerate=samplerate, channels=channels,
                        callback=audio_callback, blocksize=frames_per_block):
        while True: sd.sleep(100)

def transcriber():
    # Variables to keep track of state
    audio_buffer = []
    is_speaking = False
    silence_blocks = 0
    ghost_words = ["you", "thank you", "thanks for watching"] # words to ignore
    
    while True:
        block = audio_queue.get() # this "gets" the audio from the que
        volume = np.linalg.norm(block) * 10  # this calculates how loud our data is

        if volume > VOLUME_THRESHOLD:
            # we are talking and it will start saving the audio blocks.
            is_speaking = True
            silence_blocks = 0
            audio_buffer.append(block)
        elif is_speaking:
            silence_blocks += 1
            audio_buffer.append(block)
            if silence_blocks > SILENCE_LIMIT:
                # we've been quiet for 0.8 seconds. we are done talking.
                is_speaking = False
                # this complies all the tiny 100ms blocks together into one long audio file
                audio_data = np.concatenate(audio_buffer).flatten().astype(np.float32)
                audio_buffer = [] # empites the buffer for your next sentence
                segments, _ = whisper_model.transcribe(
                    audio_data, language="en", beam_size=5, vad_filter=True,
                    vad_parameters=dict(min_silence_duration_ms=500), condition_on_previous_text=False
                )
                text = "".join([s.text for s in segments]).strip()
                clean_text = text.lower().strip(".,!?")
                
                if clean_text and clean_text not in ghost_words:
                    text_queue.put(text)

def get_voice_command():
    return text_queue.get()

# --- LLM JSON PARSER ---
GRASPABLE_OBJECTS = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

# by setting the format =json the ollama will output json form 
intent_model = OllamaLLM(model="llama3", format="json") 
intent_template = """Map the user's physical request to EXACTLY ONE object: {allowed_classes}

Request: "{command}"

Output ONLY a valid JSON object with key "target". 
If vague, infer best object. If unmappable, output "unknown".
"""
intent_prompt = ChatPromptTemplate.from_template(intent_template)
intent_chain = intent_prompt | intent_model

# --- MAIN LOGIC ROUTER ---
def main_loop():
    threading.Thread(target=recorder, daemon=True).start()
    threading.Thread(target=transcriber, daemon=True).start()
    
    print("System active. Awaiting wake word...")
    wake_words = ["hey mara", "hey maura", "hey myra", "mara", "hey laura"] 
    reposition_phrases = ["calibrate", "reposition", "track my hand", "look at my hand"]
    
    hand_coords = "Not Set (Requires initial calibration)"
    
    while True:
        raw_user_input = get_voice_command()
        user_input = re.sub(r'[^\w\s]', '', raw_user_input).lower().strip()
        print(f"Heard: '{user_input}'")

        if user_input == "exit": break
            
        active_wake_word = next((w for w in wake_words if w in user_input), None)

        if active_wake_word:
            print("\nAction: Wake word detected.")
            
            command_clean = user_input.replace(active_wake_word, "").strip()
            
            if not command_clean:
                print("Awaiting command...")
                raw_command = get_voice_command()
                command_clean = re.sub(r'[^\w\s]', '', raw_command).lower().strip()
                print(f"Command heard: '{command_clean}'")
            else:
                print(f"Fluid command parsed: '{command_clean}'")
            
            # --- 1. CALIBRATION BLOCK ---
            if any(phrase in command_clean for phrase in reposition_phrases):
                print("Action: Triggering Hand Tracking Calibration...")
                new_coords = run_hand_tracking()
                
                if new_coords:
                    hand_coords = new_coords
                    print(f"Lock established at {hand_coords}. What should I grab?")
                else:
                    print("Action: Calibration aborted. What should I grab?")
                
                # THE FIX: Explicitly listen for the next sentence without needing the wake word!
                raw_command = get_voice_command()
                command_clean = re.sub(r'[^\w\s]', '', raw_command).lower().strip()
                print(f"Object command heard: '{command_clean}'")
            
            # --- 2. OBJECT EXECUTION BLOCK ---
            # Now it seamlessly falls straight into the LLM logic!
            print("Thinking...")
            raw_llm_output = intent_chain.invoke({
                "command": command_clean,
                "allowed_classes": ", ".join(GRASPABLE_OBJECTS)
            }).strip()
            
            try:
                clean_json = raw_llm_output.replace("```json", "").replace("```", "").strip()
                parsed_dict = json.loads(clean_json)
                target_val = parsed_dict.get("target", "unknown")
                target_object = str(target_val).lower() if target_val is not None else "unknown"
            except json.JSONDecodeError:
                target_object = "unknown"
            
            if target_object == "unknown":
                print("Error: Unrecognized command. Awaiting retry...")
                continue
            
            print(f"Target parsed: '{target_object}'. Triggering Vision System...")
            counts, found, object_coords = run_vision_system(target_object=target_object)
            
            print("\n--- EXECUTION REPORT ---")
            print(f"Hand XYZ:    {hand_coords}")
            print(f"Object XY:   {object_coords}")
            print(f"Total Found: {counts}")
            print("------------------------\nSystem active. Awaiting wake word...")

if __name__ == "__main__":
    main_loop()
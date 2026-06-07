import asyncio

from livekit.wakeword import WakeWordModel
from livekit.wakeword.inference import WakeWordListener

model = WakeWordModel(models=["hey_mara.onnx"])


async def main():
    async with WakeWordListener(model, threshold=0.3, debounce=2.0) as listener:
        print("Listening... Press Ctrl+C to stop.\n")
        while True:
            detection = await listener.wait_for_detection()
            print(f"Detected {detection.name}! (confidence={detection.confidence:.2f})")


if __name__ == "__main__":
    asyncio.run(main())
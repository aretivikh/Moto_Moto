import gtts
import pyttsx3

# Initialize the voice engine
engine = pyttsx3.init()

# Set the properties of the voice
engine.setProperty("rate", 150)  # Adjust the speed of the voice
engine.setProperty("volume", 1)  # Adjust the volume of the voice

# Provide the text that you want to generate speech from
text = "Test is completed. Return HATS to 0°"
# text = "Now I will rotate HATS and repeat test under different angles."
# text = "Now I will start SSR test, side speech rejection. Speaker will play wake word at position 0°"
# text = "the first test is accuracy, HATS will say wake word and supernova should recognize it. "




# make request to google to get synthesis
tts = gtts.gTTS(text)
# save the audio file
tts.save("4.mp3")
# play the audio file

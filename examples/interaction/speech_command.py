"""
Windows
You can just pip install it:

pip3 install pyaudio
Linux
You need to first install the dependencies:

sudo apt-get install python-pyaudio python3-pyaudio
pip3 install pyaudio
MacOS
You need to first install portaudio, then you can just pip install it:

brew install portaudio
pip3 install pyaudio
"""


from gtts import gTTS
from playsound import playsound
# Language in which you want to convert
language = 'en'

# Passing the text and language to the engine, 
# here we have marked slow=False. Which tells 
# the module that the converted audio should 
# have a high speed
import speech_recognition as sr

r = sr.Recognizer()
with sr.Microphone() as source:
    # read the audio data from the default microphone
    print("start recording")
    audio_data = r.listen(source) #r.record(source, duration=3)
    print("Recognizing...")
    # convert speech to text
    text = r.recognize_google(audio_data)
    print(text)

myobj = gTTS(text=text, lang=language, slow=False)

# Saving the converted audio in a mp3 file named
# welcome
myobj.save("welcome.mp3")

# Playing the converted file
playsound("welcome.mp3")

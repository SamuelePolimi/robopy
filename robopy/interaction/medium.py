import speech_recognition as sr
from gtts import gTTS
from playsound import playsound


class Medium:

    def __init__(self):
        pass

    def signal(self, text: str):
        raise NotImplemented()

    def read(self, text: str):
        raise NotImplemented()


class Console(Medium):

    def __init__(self):
        Medium.__init__(self)

    def signal(self, text: str):
        print(text)

    def read(self, text: str):
        return input(text)


class Audio(Medium):

    def __init__(self, language='en', timeout=200, mic_index=0, console_transcript=True, temporary_folder='/tmp/'):
        Medium.__init__(self)
        self._language = language
        self._timeout = timeout
        self._mic_index = mic_index
        self._console_transcript = console_transcript
        self._temporary = temporary_folder + "temp.mp3"
        self._recognizer = r.Recognizer()

    def signal(self, text: str):
        myobj = gTTS(text=text, lang=self._language, slow=False)

        myobj.save(self._temporary)
        playsound(self._temporary)
        if self._console_transcript:
            print("Out: %s" % text)

    def read(self, text: str):
        if text is not None or text != "":
            self.signal(text)

        text = "Timed out or not understood"
        with sr.Microphone() as source:
            audio_data = self._recognizer.listen(source)  # r.record(source, duration=3)
            text = self._recognizer.recognize_google(audio_data)

        if self._console_transcript:
            print("In: %s" % text)

        return text






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

from robopy.interaction import Console, Audio, Interaction

def play(mode):
    interaction = Interaction(mode)
    interaction.signal("Hi there!")
    name = interaction.read("Tell me your name: ")
    if interaction.yes_no_question("Is your name %s?" % name):
        interaction.signal("Cool name!")
    else:
        interaction.signal("Sorry for not getting it!")


"""
Example of the classic interaction with the console
"""

play(Console())

"""
Example of audio interaction
"""

play(Audio())
from robopy.interaction.medium import Medium, Console, Audio


class Interaction:

    def __init__(self, medium: Medium):
        self._medium = medium

    def yes_no_question(self, text):

        if type(self._medium) is Console:
            positive = ['y', 'Y']
            negative = ['n', 'N']
            question = text + " (y/n): "
        else:
            positive = ['yes', 'yeah']
            negative = ['no', 'nope']
            question = text + " yes or no?"

        while True:
            answer = self.read(question)
            if answer in positive:
                return True
            elif answer in negative:
                return False

            if type(self._medium) is Console:
                self._medium.signal("Type only 'y', Y' or 'n', 'N'")
            else:
                self._medium.signal("Answer not understood. Answer only with yes or no.")

    def ready(self):
        if type(self._medium) is Console:
            return self.read("Press ENTER when ready: ")
        else:
            return self.read("When are you done, say: done.")

    def signal(self, text: str):
        return self._medium.signal(text)

    def read(self, text: str):
        return self._medium.read(text)
